// Calibration Test
// Zuhayr Loonat - EEE4022S - Thesis - Salinity Probe

// ===== Includes =====
// --- For Code ---
#include <stdbool.h>

// --- For ESP32 ---
#include "freertos/FreeRTOS.h"          // Core RTOS definitions
#include "freertos/task.h"              // xTaskCreate, vTaskDelay, etc.
#include "driver/gpio.h"                // GPIO input/output
#include "esp_log.h"                    // Logging functions

// --- My Components ---
#include "i2c_driver.h"
#include "rs485_driver.h"
#include "adc_driver.h"
#include "mcp4725_driver.h"
#include "ms5837_driver.h"

// for components (not needed coz i made seperate files for them)
/*
#include "driver/i2c.h"                 // I²C master/slave driver
#include "driver/uart.h"                // UART/RS485 driver
#include "driver/adc.h"                 // ADC channels
#include "esp_adc_cal.h"                // ADC calibration library
*/


// ===== Definitions =====      

// --- ADC Input Pins ---       
#define ADC_UNB_CHN  ADC1_CHANNEL_0     // ADC input from DAC output
#define ADC_DAC_CHN  ADC1_CHANNEL_1     // ADC input from DAC output after Buffer Op-Amp
#define ADC_AMP_CHN  ADC1_CHANNEL_2     // ADC input before R2
#define ADC_SGN_CHN  ADC1_CHANNEL_3     // ADC input from x11 ampl (drop over R2)

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
float calibrationFactor2 = 1;
float calibrationFactor3 = 1;
float calibrationFactor4 = 0.81;
// Sensor Config
//ms5837_config_t sensor_config;

// ===== Start of app_main =====
void app_main() {

} // end of app_main


// ===== Functions =====

