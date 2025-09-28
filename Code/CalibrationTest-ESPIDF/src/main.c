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
#define I2C_PORT     I2C_NUM_0      
#define I2C_FREQ     100000             // 100 kHz
#define SDA_PIN      8                  // SDA GPIO PIN 
#define SCL_PIN      9                  // SCL GPIO PIN
#define MCP4725_ADDR 0x60               // MCP4725A0 default address
#define MS5837_ADDR  0x77               // MS5837 address

// --- DAC Parameters ---       
#define DAC_MAX_5V  4095                // 12-bit range
#define DAC_MAX_3V3 2200                // Limit for 3V out when powered at 5V

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
#define ADC_ATTEN    ADC_ATTEN_DB_0     // maps ~0-3.3 V to full scale (attenuate for higher voltages, 0:k=100%, 2.5:k=75%, 6:k=50%, 12:k=25%)
#define ADC_WIDTH    ADC_BITWIDTH_13    // 13-bit resolution

// --- RS485 ---        
#define RS485_RO    39                  // RS485 RO Pin
#define RS485_RE_DE 40                  // RS485 RE/DE Pin
#define RS485_DI    41                  // RS485 RI Pin

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

// ===== Start of app_main =====
void app_main() {

} // end of app_main


// ===== Functions =====

// --- DAC Functions ---
/*
    The DAC expects 3 bytes: 
    [ Command + PD bits ] [ DAC MSB ] [ DAC LSB ]
    For a fast DAC update: command = 0x40 (writes DAC register only, no EEPROM)
    12 Bit Value: 0-4095
    Power-down modes: Bits PD1/PD0 in the command byte let you put the output in high-impedance or pull-down states.
*/

/** 
 *  @brief Initialises I²C
 */
static void i2c_master_init(void)
{
    // Configure I2C parameters
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,                // Set as I2C master
        .sda_io_num = SDA_PIN,                  // SDA pin number
        .scl_io_num = SCL_PIN,                  // SCL pin number
        .sda_pullup_en = GPIO_PULLUP_ENABLE,    // Enable pull-up on SDA
        .scl_pullup_en = GPIO_PULLUP_ENABLE,    // Enable pull-up on SCL
        .master.clk_speed = I2C_FREQ,           // Set I2C clock frequency
    };
    // Apply I2C configuration to the specified port
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    // Install I2C driver for the port
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
}

/**
 *  @brief Sets the output voltage of the MCP4725 DAC.
 *  @param value 12-bit value (0-4095) representing the desired output voltage.
 *  @return esp_err_t Error code from I²C write operation.
 */
static esp_err_t mcp4725_set_voltage(uint16_t value)
{
    uint8_t write_buf[3];
    write_buf[0] = 0x40;                       // Command: Fast mode, update DAC register
    write_buf[1] = value >> 4;                 // Upper 8 bits of the 12-bit value
    write_buf[2] = (value & 0x0F) << 4;        // Lower 4 bits, left-aligned in byte

    // Send the buffer to the MCP4725 via I2C
    return i2c_master_write_to_device(
        I2C_PORT,                              // I2C port number
        MCP4725_ADDR,                          // I2C address of MCP4725
        write_buf,                             // Data buffer to send
        sizeof(write_buf),                     // Number of bytes to send (3)
        pdMS_TO_TICKS(100)                     // Timeout in FreeRTOS ticks
    );
}

// --- ADC Functions ---
/*
Resolution: 13 Bit: 0-8191, or 12 bit: 0-4095
NEVER EXCEDED 3V3
Vdata = Vref/k * data/((2^bits)-1) where k is attenuation
*/

/**
 * @brief Initialize ADC with proper configuration and calibration
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