#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/adc.h>     


#define SLEEP_TIME_MS          1000
#define RECEIVE_BUFF_SIZE      10
#define RECEIVE_TIMEOUT        100

#include <hal/nrf_saadc.h>
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_CHANNEL_ID 1
#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1

// Global ADC device instance
const struct device *adc_dev = DEVICE_DT_GET_ONE(nordic_nrf_saadc);

/**
 * @struct IoModuleData
 * @brief Holds all input/output module data including state of LEDs, buttons and ADC values.
 */
typedef struct {
    uint8_t led_state[4];  // States of 4 LEDs
    uint8_t button_state[4];  // States of 4 buttons
    int16_t an_raw;  // Raw analog sensor value
    int an_val;  // Processed analog sensor value as integer (e.g., milli-degrees Celsius)
} IoModuleData;

/**
 * @struct RealTimeDatabase
 * @brief Struct to hold real-time data and a mutex for thread-safe access.
 */
typedef struct {
    IoModuleData data;  ///< Embedded structure to hold module data.
    struct k_mutex lock;  ///< Mutex to protect access to data.
} RealTimeDatabase;

static RealTimeDatabase rtdb;

typedef struct {
    int16_t raw_value;
    float temperature;
} SensorData;


static const struct adc_channel_cfg my_channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_ID,
    .input_positive = ADC_CHANNEL_INPUT
};

// GPIO device tree specs for LEDs and buttons
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);

static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static const struct gpio_dt_spec button3 = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);
static const struct gpio_dt_spec button4 = GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios);

void button_thread(void *p1, void *p2, void *p3);
void led_thread(void *p1, void *p2, void *p3);



K_THREAD_DEFINE(button_tid, 512, button_thread, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(led_tid, 1024, led_thread, NULL, NULL, NULL, 7, 0, 0);

void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data);

K_THREAD_STACK_DEFINE(uart_stack, 1024);
struct k_thread uart_thread_data;

K_THREAD_STACK_DEFINE(adc_stack, 1024);  // Define stack for ADC thread
struct k_thread adc_thread_data;         // Define thread data for ADC thread

// Define message queues
K_MSGQ_DEFINE(msgq_adc_raw, sizeof(int16_t), 10, 4); // Queue for raw ADC data
K_MSGQ_DEFINE(msgq_temperature, sizeof(float), 10, 4); // Queue for processed temperature data
K_MSGQ_DEFINE(msgq_sensor_data, sizeof(SensorData), 10, 4); // Queue for sensor data including raw and temperature



// Thread data and stacks
K_THREAD_STACK_DEFINE(sensor_stack, 1024);
struct k_thread sensor_thread_data;

K_THREAD_STACK_DEFINE(process_stack, 1024);
struct k_thread process_thread_data;

K_THREAD_STACK_DEFINE(database_stack, 1024);
struct k_thread database_thread_data;

// Function prototypes
void sensor_reading_thread(void *p1, void *p2, void *p3);
void data_processing_thread(void *p1, void *p2, void *p3);
void database_thread(void *p1, void *p2, void *p3);





/**
 * @brief Configures buttons and LEDs to known states and settings.
 */
static void configure_buttons_and_leds(void) {
    gpio_pin_configure_dt(&button1, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&button2, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&button3, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&button4, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led3, GPIO_OUTPUT_ACTIVE);
}

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
static uint8_t tx_buf[] = "xxxxxxxxxxxxxx Welcome xxxxxxxxxxxxxx\n\r";
static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};


/**
 * @brief UART event callback function to handle incoming data and control device peripherals.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param evt Data structure containing event details.
 * @param user_data Additional data, unused in this callback.
 */
void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data) {
    static char output[64]; // Buffer to store output string

    switch (evt->type) {
        case UART_RX_RDY:
            for (int i = 0; i < evt->data.rx.len; i++) {
                uint8_t cmd = evt->data.rx.buf[evt->data.rx.offset + i];

                if (cmd >= '1' && cmd <= '4') {
                    int led_idx = cmd - '1';
                    k_mutex_lock(&rtdb.lock, K_FOREVER);
                    rtdb.data.led_state[led_idx] ^= 1; // Toggle LED state in the database
                    k_mutex_unlock(&rtdb.lock);
                    snprintf(output, sizeof(output), "Toggle LED %d \r\n", led_idx + 1);
                } else if (cmd >= '5' && cmd <= '8') {
                    int button_idx = cmd - '5';
                    k_mutex_lock(&rtdb.lock, K_FOREVER);
                    int state = rtdb.data.button_state[button_idx];
                    k_mutex_unlock(&rtdb.lock);
                    snprintf(output, sizeof(output), "Button %d state: %d\r\n", button_idx + 1, state);
                } else if (cmd == '9') {
                    k_mutex_lock(&rtdb.lock, K_FOREVER);
                    int raw_value = rtdb.data.an_raw;
                    k_mutex_unlock(&rtdb.lock);
                    snprintf(output, sizeof(output), "Raw sensor value: %d\r\n", raw_value);
                } else if (cmd == '0') {
                    k_mutex_lock(&rtdb.lock, K_FOREVER);
                    int processed_value = rtdb.data.an_val;
                    k_mutex_unlock(&rtdb.lock);
                    snprintf(output, sizeof(output), "Processed sensor value: %d  Celsius\r\n", processed_value);
                } else {
                    continue; // Ignore unrecognized commands
                }

                uart_tx(dev, output, strlen(output), SYS_FOREVER_MS);
            }
            break;
        case UART_RX_DISABLED:
            uart_rx_enable(dev, rx_buf, sizeof(rx_buf), RECEIVE_TIMEOUT);
            break;
        default:
            break;
    }
}

/**
 * @brief Thread function to control LED states based on data in the shared database.
 *
 * This thread periodically checks if the LED states stored in the real-time database have changed.
 * If a change is detected, it updates the physical state of the LEDs to reflect these changes.
 * The function uses a mutex to synchronize access to the shared data, ensuring thread safety.
 *
 * @param p1 Unused parameter.
 * @param p2 Unused parameter.
 * @param p3 Unused parameter.
 *
 * @note This function is designed to run indefinitely as a thread in the system.
 *       It uses a mutex to ensure that access to shared resources is thread-safe.
 */
void led_thread(void *p1, void *p2, void *p3) {
    const struct gpio_dt_spec *leds[] = {&led0, &led1, &led2, &led3};
    uint8_t current_led_states[4] = {0};

    while (1) {
        k_mutex_lock(&rtdb.lock, K_FOREVER);
        for (int i = 0; i < 4; i++) {
            if (rtdb.data.led_state[i] != current_led_states[i]) {
                gpio_pin_set_dt(leds[i], rtdb.data.led_state[i]);
                current_led_states[i] = rtdb.data.led_state[i]; // Update current state to match the database
            }
        }
        k_mutex_unlock(&rtdb.lock);
        k_msleep(100);  // Sleep for some time before checking again
    }
}

/**
 * @brief Thread function to monitor the state of buttons and update the shared database.
 *
 * This thread continuously checks the state of each configured button. If a change in the state
 * of any button is detected, the new state is updated in the shared real-time database. This
 * function ensures that any changes in the button states are captured and stored accurately,
 * allowing other parts of the program to react to user input.
 *
 * @param p1 Unused parameter.
 * @param p2 Unused parameter.
 * @param p3 Unused parameter.
 *
 * @note This function runs indefinitely as a thread, repeatedly checking button states and
 *       updating the shared database. It employs a mutex to synchronize access to the shared
 *       data to prevent data races and ensure consistency.
 */
void button_thread(void *p1, void *p2, void *p3) {
    while (1) {
        k_mutex_lock(&rtdb.lock, K_FOREVER);
        bool current_state;

        // Check button 1 state and print if changed
        current_state = gpio_pin_get_dt(&button1);
        if (current_state != rtdb.data.button_state[0]) {
            rtdb.data.button_state[0] = current_state;
            
        }

        // Check button 2 state and print if changed
        current_state = gpio_pin_get_dt(&button2);
        if (current_state != rtdb.data.button_state[1]) {
            rtdb.data.button_state[1] = current_state;
            
        }

        // Check button 3 state and print if changed
        current_state = gpio_pin_get_dt(&button3);
        if (current_state != rtdb.data.button_state[2]) {
            rtdb.data.button_state[2] = current_state;
            
        }

        // Check button 4 state and print if changed
        current_state = gpio_pin_get_dt(&button4);
        if (current_state != rtdb.data.button_state[3]) {
            rtdb.data.button_state[3] = current_state;
            
        }
        //printk("button thread\n");
        k_mutex_unlock(&rtdb.lock);
        k_msleep(100);  // 
    }
}

static uint16_t adc_sample_buffer[1];  // Single sample buffer


/**
 * @brief Reads an analog value from the ADC channel specified in the configuration.
 *
 * This function configures an ADC sequence with predefined settings such as channel, buffer,
 * buffer size, and resolution. It initiates an ADC read using the configured sequence
 * and returns the result of the read operation.
 *
 * @param adc_dev Pointer to the ADC device structure.
 * @return int Returns 0 if the ADC read is successful, otherwise returns a negative error code.
 *
 * @note The ADC configuration, including gain, reference voltage, and acquisition time, is set
 *       elsewhere in the program and affects the accuracy and range of the ADC read.
 */
static int read_adc(const struct device *adc_dev)
{
    struct adc_sequence sequence = {
        .channels    = BIT(ADC_CHANNEL_ID),
        .buffer      = adc_sample_buffer,
        .buffer_size = sizeof(adc_sample_buffer),
        .resolution  = ADC_RESOLUTION,
    };
    return adc_read(adc_dev, &sequence);
}

/*void adc_thread(void *p1, void *p2, void *p3) {
    const struct device *adc_dev = DEVICE_DT_GET_ONE(nordic_nrf_saadc);
    adc_channel_setup(adc_dev, &my_channel_cfg);

    while (1) {
        if (read_adc(adc_dev) == 0) {
            k_mutex_lock(&rtdb.lock, K_FOREVER);
            rtdb.data.an_raw = adc_sample_buffer[0];
            // Correct calculation for voltage and temperature
            float voltage = (rtdb.data.an_raw / 1023.0f) * 3.0f; // Scaling the raw value to voltage
            rtdb.data.an_val = 60 * (voltage - 1); // Calculating temperature from voltage
            k_mutex_unlock(&rtdb.lock);

            char buffer[128]; // Buffer for formatted string
            int length = snprintf(buffer, sizeof(buffer), "ADC Raw: %d, Voltage: %.2f V, Temperature: %.2f °C\n",
                                  rtdb.data.an_raw, voltage, rtdb.data.an_val);

            if (length > 0 && length < sizeof(buffer)) {
                printk("%s", buffer);
            } else {
                printk("Error: Buffer overflow or snprintf failed\n");
            }
        }
        k_msleep(1000);
    }
}*/

 /**
 * @brief Thread function to continuously read sensor data using ADC.
 *
 * This thread initializes the ADC device and continuously reads the ADC values,
 * posting the raw sensor data to a message queue. This function aims to sample sensor data at a
 * regular interval of one second.
 *
 * @param p1 Unused parameter.
 * @param p2 Unused parameter.
 * @param p3 Unused parameter.
 */
void sensor_reading_thread(void *p1, void *p2, void *p3) {
    const struct device *adc_dev = DEVICE_DT_GET_ONE(nordic_nrf_saadc);
    adc_channel_setup(adc_dev, &my_channel_cfg);

    while (1) {
        int16_t raw_value;
        if (read_adc(adc_dev) == 0) {
            raw_value = adc_sample_buffer[0];
            k_msgq_put(&msgq_adc_raw, &raw_value, K_FOREVER);
            //printk("Sensor reading\n");
        }
        k_msleep(1000);  // Sampling every second
    }
}

/**
 * @brief Thread function to process raw ADC data from a message queue and convert it to meaningful values.
 *
 * This thread retrieves raw ADC data from a message queue, converts it to voltage, and calculates
 * the temperature from the voltage. It then stores the processed data in another message queue for
 * further usage.
 *
 * @param p1 Unused parameter.
 * @param p2 Unused parameter.
 * @param p3 Unused parameter.
 */
void data_processing_thread(void *p1, void *p2, void *p3) {
    int16_t raw_value;
    SensorData data;
    while (1) {
        k_msgq_get(&msgq_adc_raw, &raw_value, K_FOREVER);
        float voltage = (raw_value / 1023.0f) * 3.0f;  // Convert ADC value to voltage
        //data.temperature = 60 * (voltage - 1);        // Convert voltage to temperature
        data.temperature = (int)(60000 * (voltage - 1));  // Example for scaling
        data.raw_value = raw_value;                   // Store raw value
        k_msgq_put(&msgq_sensor_data, &data, K_FOREVER);
        //printk("Data_processing thread\n");
    }
}


/**
 * @brief Thread function to store processed sensor data into a shared database.
 *
 * This thread retrieves processed sensor data from a message queue and stores it in a global
 * structure protected by a mutex. This ensures that the data is accessible across different parts
 * of the program in a thread-safe manner.
 *
 * @param p1 Unused parameter.
 * @param p2 Unused parameter.
 * @param p3 Unused parameter.
 */
void database_thread(void *p1, void *p2, void *p3) {
    SensorData data;
    while (1) {
        k_msgq_get(&msgq_sensor_data, &data, K_FOREVER);
        k_mutex_lock(&rtdb.lock, K_FOREVER);
        rtdb.data.an_raw = data.raw_value;
        rtdb.data.an_val = data.temperature;  // Store the latest temperature in the shared data
        k_mutex_unlock(&rtdb.lock);
        //printk("database thread\n");
    }
}

//K_THREAD_DEFINE(adc_tid, 1024, adc_thread, NULL, NULL, NULL, 7, 0, 0);

/**
 * @brief Main function of the Zephyr application.
 *
 * This function performs the initial setup of the application. It configures buttons and LEDs,
 * initializes mutexes, checks device readiness, sets up UART communication, and creates
 * necessary threads for UART handling, sensor data reading, data processing, and database updates.
 *
 * @return int Returns 0 on success, and non-zero on error.
 */
int main(void) {
    configure_buttons_and_leds();
    k_mutex_init(&rtdb.lock);
    memset(&rtdb.data, 0, sizeof(rtdb.data));  // Initialize all states to off

    if (!device_is_ready(uart)) {
        printk("UART device not ready\n");
        return 1;
    }

    if (!device_is_ready(adc_dev)) {
        printk("ADC device not ready\n");
        return 1;
    }

    int ret = uart_callback_set(uart, uart_callback, NULL);
    if (ret) {
        printk("Failed to set UART callback\n");
        return 1;
    }

    ret = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_MS);
    if (ret) {
        printk("UART transmission failed\n");
        return 1;
    }

    ret = uart_rx_enable(uart, rx_buf, sizeof(rx_buf), RECEIVE_TIMEOUT);
    if (ret) {
        return 1;
    }

    k_thread_create(&uart_thread_data, uart_stack, K_THREAD_STACK_SIZEOF(uart_stack),
                    uart_callback, NULL, NULL, NULL, 6, 0, K_NO_WAIT);

    //k_thread_create(&adc_thread_data, adc_stack, K_THREAD_STACK_SIZEOF(adc_stack),
                    //adc_thread, NULL, NULL, NULL, 6, 0, K_NO_WAIT);                
    
    // Create threads for sensor reading, data processing, and database
   // Thread creation
    k_thread_create(&sensor_thread_data, sensor_stack, K_THREAD_STACK_SIZEOF(sensor_stack), sensor_reading_thread, NULL, NULL, NULL, 7, 0, K_NO_WAIT);
    k_thread_create(&process_thread_data, process_stack, K_THREAD_STACK_SIZEOF(process_stack), data_processing_thread, NULL, NULL, NULL, 6, 0, K_NO_WAIT);
    k_thread_create(&database_thread_data, database_stack, K_THREAD_STACK_SIZEOF(database_stack), database_thread, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
    
        k_msleep(SLEEP_TIME_MS);
    }
