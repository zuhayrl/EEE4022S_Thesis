// Calibration Test
// Zuhayr Loonat - EEE4022S - Thesis - Salinity Probe

// Includes
#include "freertos/FreeRTOS.h"          // Core RTOS definitions
#include "freertos/task.h"              // xTaskCreate, vTaskDelay, etc.
#include "driver/gpio.h"                // GPIO input/output
#include "driver/i2c.h"                 // I²C master/slave driver
#include "driver/uart.h"                // UART/RS485 driver
#include "driver/adc.h"                 // ADC channels
#include "esp_adc_cal.h"                // ADC calibration library
#include "esp_log.h"                    // Logging functions


// ===== Definitions =====      
// --- I²C ---      
#define MS5837_ADDR  0x77               // MS5837 address

// --- DAC Parameters ---       

// --- ADC Input Pins ---       
//#define ADC_UNB_PIN  1                // ADC input from DAC output
#define ADC_UNB_CHN  ADC1_CHANNEL_0
//#define ADC_DAC_PIN  2                // ADC input from DAC output after Buffer Op-Amp
#define ADC_DAC_CHN  ADC1_CHANNEL_1
//#define ADC_AMP_PIN  3                // ADC input before R2
#define ADC_AMP_CHN  ADC1_CHANNEL_2
//#define ADC_SGN_PIN  4                // ADC input from x11 ampl (drop over R2)
#define ADC_SGN_CHN  ADC1_CHANNEL_3
// ADC Parameters
#define ADC_ATTEN    ADC_ATTEN_DB_0     // Maps ~0-3.3 V to full scale (attenuate for higher voltages, 0:k=100%, 2.5:k=75%, 6:k=50%, 12:k=25%)
#define ADC_WIDTH    ADC_BITWIDTH_13    // 13-bit resolution

// --- RS485 ---    
// Pinouts    
#define RS485_RO    39                  // RS485 RO Pin
#define RS485_RE_DE 40                  // RS485 RE/DE Pin
#define RS485_DI    41                  // RS485 RI Pin
// UART Config
#define RS485_UART_NUM      UART_NUM_1  // UART Number
#define RS485_BAUD_RATE     9600        // Adjust as needed
#define RS485_BUF_SIZE      1024        // RX/TX buffer size
#define RS485_QUEUE_SIZE    20          // Event queue size
// Timing (N.B: this is in uSeconds) needed since this is half-duplex
#define RS485_TX_DELAY_US   50          // Delay before transmit
#define RS485_RX_DELAY_US   100         // Delay after transmit before receive

// --- Switches ---     
// R1 Resistor Switch       
#define SW_R1_100    5                  // R1 - 100 Ohm switch
#define SW_R1_1k     6                  // R1 - 100 Ohm switch
#define SW_R1_10k    7                  // R1 - 100 Ohm switch
#define SW_Calib    21                  // Calibration Resistor switch
// Gold Pads Switch     
#define SW_AuP      10                  // Au+ On
#define SW_AuN      11                  // Au- On
#define SW_AuP_GND  12                  // Au+ GND
#define SW_AuN_GND  13                  // Au- GND
// Guard/Shield Switch      
#define SW_AuP_ShP  33                  // AU+ Shield+
#define SW_AuN_ShP  34                  // AU- Shield+
#define SW_AuP_ShN  35                  // AU+ Shield-
#define SW_AuN_ShN  36                  // AU- Shield-
// ADC Switch       
#define SW_ADC1 14                      // ADC IO 1 Switch
#define SW_ADC2 15                      // ADC IO 2 Switch
#define SW_ADC3 16                      // ADC IO 3 Switch
#define SW_ADC4 17                      // ADC IO 4 Switch

// ===== Other Global Vars =====
// Calibration factor for ADC
float calibrationFactor1 = 0.875; 
float calibrationFactor4 = 0.81;
static esp_adc_cal_characteristics_t *adc_chars;
// RS485
static QueueHandle_t rs485_queue;
static bool rs485_initialised = false;

// ===== Start of app_main =====
void app_main() {

} // end of app_main


// ===== Functions =====

// --- ADC Functions ---
/*
Resolution: 13 Bit: 0-8191, or 12 bit: 0-4095
NEVER EXCEDED 3V3
Vdata = Vref/k * data/((2^bits)-1) where k is attenuation
*/

/**
 * @brief initialise ADC with proper configuration and calibration
 *        When to call: Once during initialization, after I2C init.
 * @return esp_err_t Error code
 */
static esp_err_t adc_init(void){

    // Configure ADC1 (Channels 0-3) for specified width and attenuation
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_UNB_CHN, ADC_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_DAC_CHN, ADC_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_AMP_CHN, ADC_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_SGN_CHN, ADC_ATTEN));

    // Characterize ADC for voltage calculation
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, 1100, adc_chars);

    // Print calibration information
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    }
    else {
        printf("Characterized using Default Vref\n");
    }

    return ESP_OK;

}

/**
 * @brief Read ADC value from specified channel and return calibrated voltage
 * @param channel ADC channel to read from (ADC_UNB_CHN, ADC_DAC_CHN, etc.)
 * @param samples Number of samples to average (default: 64) (higher - slower)
 * @return float bit Voltage reading in volts
 */
static float adc_read_voltage(adc1_channel_t channel, int samples){
    if (samples <= 0) samples = 64; // Default sampling
    uint32_t adc_reading = 0;

    // Take multiple samples for better accuracy
    for (int i = 0; i < samples; i++) {
        adc_reading += adc1_get_raw(channel);
    }

    adc_reading /= samples;

    // Convert adc_reading to voltage in mV then to volts
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

    return voltage_mv / 1000.0; // Convert mV to V
}

/**
 * @brief Read raw ADC value from specified channel (No voltage conversion)
 * @param channel ADC channel to read from
 * @param samples Number of samples to average (default: 64)
 * @return int Raw ADC reading (0-4095 for 12-bit)(0-8191 for 13-bit)
 */
static int adc_read_raw(adc1_channel_t channel, int samples){

    if (samples <= 0) samples = 64; // Default sampling
    uint32_t adc_reading = 0;

    // Take multiple samples for better accuracy
    for (int i = 0; i < samples; i++) {
        adc_reading += adc1_get_raw(channel);
    }

    return adc_reading / samples;
}


// --- RS485 Functions ---


// Initilise RS485
/**
 * @brief initialise RS485 communication using SP3485EN IC
 * @param baud_rate Baud rate for communication (default: 9600)
 * @return esp_err_t Error code
 */
static esp_err_t rs485_init(uint32_t baud_rate){
    
    // Check if already initialised
    if (rs485_initialised) {
        return ESP_OK;
    }

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(RS485_UART_NUM, RS485_BUF_SIZE, RS485_BUF_SIZE, RS485_QUEUE_SIZE, &rs485_queue, 0));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(RS485_UART_NUM, &uart_config));

    // Set UART pins (TX, RX, RTS, CTS)
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART_NUM, RS485_DI, RS485_RO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Configure RE/DE pin as output (it's initially in receive mode)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << RS485_RE_DE),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Set to receive mode (RE/DE = LOW)
    rs485_set_receive_mode();
    rs485_initialised = true;
    printf("RS485 initialised at %d baud\n", baud_rate);
    return ESP_OK;
}

// Modes (Rx/Rx)
/**
 * @brief Set SP3485EN to transmit mode
 */
static void rs485_set_transmit_mode(void){
    gpio_set_level(RS485_RE_DE, 1); // RE/DE = HIGH (transmit)
    esp_rom_delay_us(RS485_TX_DELAY_US); // Wait for transceiver switching
}

/**
 * @brief Set SP3485EN to receive mode  
 */
static void rs485_set_receive_mode(void){
    gpio_set_level(RS485_RE_DE, 0); // RE/DE = LOW (receive)
    esp_rom_delay_us(RS485_RX_DELAY_US); // Wait for transceiver switching
}

/**
 * @brief Get current RS485 direction
 * @return true if in transmit mode, false if in receive mode
 */
static bool rs485_is_transmit_mode(void){
    return gpio_get_level(RS485_RE_DE) == 1;
}

// Send/Receive
/**
 * @brief Send data over RS485 bus
 * @param data Pointer to data buffer
 * @param length Number of bytes to send
 * @param timeout_ms Timeout in milliseconds
 * @return int Number of bytes sent, -1 on error
 */
static int rs485_send_data(const uint8_t *data, size_t length, uint32_t timeout_ms){

    // Check if initialised
    if (!rs485_initialised || data == NULL || length == 0) {
        return -1;
    }

    // Clear any pending RX data (flush UART)
    uart_flush_input(RS485_UART_NUM);

    // Switch to transmit mode
    rs485_set_transmit_mode();

    // Send data
    int bytes_sent = uart_write_bytes(RS485_UART_NUM, data, length);

    // Wait for transmission to complete
    uart_wait_tx_done(RS485_UART_NUM, pdMS_TO_TICKS(timeout_ms));

    // Switch back to receive mode
    rs485_set_receive_mode();

    return bytes_sent;
}

/**
 * @brief Receive data from RS485 bus
 * @param buffer Pointer to receive buffer
 * @param buffer_size Size of receive buffer
 * @param timeout_ms Timeout in milliseconds
 * @return int Number of bytes received, -1 on error
 */
static int rs485_receive_data(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms){

    // Check if initialised
    if (!rs485_initialised || buffer == NULL || buffer_size == 0) {
        return -1;
    }

    // Ensure we're in receive mode
    if (rs485_is_transmit_mode()) {
        rs485_set_receive_mode();
    }

    // Read data with timeout
    int bytes_received = uart_read_bytes(RS485_UART_NUM, buffer, buffer_size - 1, pdMS_TO_TICKS(timeout_ms));

    if (bytes_received > 0) {
        buffer[bytes_received] = '\0'; // Null terminate for string operations
    }

    return bytes_received;
}

/**
 * @brief Send string over RS485 bus
 * @param str Null-terminated string to send
 * @param timeout_ms Timeout in milliseconds
 * @return int Number of bytes sent, -1 on error
 */
static int rs485_send_string(const char *str, uint32_t timeout_ms){
    if (str == NULL) {
        return -1;
    }
    return rs485_send_data((const uint8_t *)str, strlen(str), timeout_ms);
}

/**
 * @brief Send command and wait for response
 * @param command Command string to send
 * @param response Buffer for response
 * @param response_size Size of response buffer
 * @param timeout_ms Timeout for response in milliseconds
 * @return int Number of bytes received, -1 on error
 */
static int rs485_send_command(const char *command, char *response, size_t response_size, uint32_t timeout_ms){

    // Check if there is a command and place to store response
    if (command == NULL || response == NULL) {
        return -1;
    }

    // Send command
    int sent = rs485_send_string(command, 1000);
    if (sent <= 0) {
        return -1;
    }

    // Wait for response
    return rs485_receive_data((uint8_t *)response, response_size, timeout_ms);
}

// Helper functions
/**
 * @brief Check if data is available to read
 * @return size_t Number of bytes available
 */
static size_t rs485_available(void){
    
    if (!rs485_initialised) {
        return 0;
    }

    size_t bytes_available;
    uart_get_buffered_data_len(RS485_UART_NUM, &bytes_available);
    return bytes_available;
}

/**
 * @brief Flush RS485 buffers (clear Rx data)
 */
static void rs485_flush(void){

    if (rs485_initialised) {
        uart_flush(RS485_UART_NUM);
    }
}