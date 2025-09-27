// Calibration Test
// Zuhayr Loonat - EEE4022S - Thesis - Salinity Probe

// Includes
#include "freertos/FreeRTOS.h"          // Core RTOS definitions
#include "freertos/task.h"              // xTaskCreate, vTaskDelay, etc.
#include "driver/gpio.h"                // GPIO input/output
#include "driver/i2c.h"                 // I²C master/slave driver
#include "driver/uart.h"                // UART/RS485 driver
#include "driver/adc.h"                 // ADC channels


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
#define ADC_UNB_PIN  1                  // ADC input from DAC output
#define ADC_DAC_PIN  2                  // ADC input from DAC output after Buffer Op-Amp
#define ADC_AMP_PIN  3                  // ADC input before R2
#define ADC_SGN_PIN  4                  // ADC input from x11 ampl (drop over R2)

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
 *  @brief Initialises I2C
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
 *  @return esp_err_t Error code from I2C write operation.
 */
static esp_err_t mcp4725_set_voltage(uint16_t value)
{
    uint8_t write_buf[3];
    write_buf[0] = 0x40;                       // Command: Fast mode, update DAC register
    write_buf[1] = value >> 4;                 // Upper 8 bits of the 12-bit value
    write_buf[2] = (value & 0x0F) << 4;        // Lower 4 bits, left-aligned in byte

    // Send the buffer to the MCP4725 via I2C
    return i2c_master_write_to_device(
        I2C_PORT,           // I2C port number
        MCP4725_ADDR,       // I2C address of MCP4725
        write_buf,          // Data buffer to send
        sizeof(write_buf),  // Number of bytes to send (3)
        pdMS_TO_TICKS(100)  // Timeout in FreeRTOS ticks
    );
}
