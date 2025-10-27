/*  Header file for ADC Driver
    Salinity Probe
    Zuhayr Loonat
    EEE4022S - 2025
*/

// ===== Includes =====
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_err.h"

// ===== Defines =====
// ADC Channels
#define ADC_UNB_CHN  ADC1_CHANNEL_0     // ADC input from DAC output
#define ADC_DAC_CHN  ADC1_CHANNEL_1     // ADC input from DAC output after Buffer Op-Amp
#define ADC_AMP_CHN  ADC1_CHANNEL_2     // ADC input before R2
#define ADC_SGN_CHN  ADC1_CHANNEL_3     // ADC input from x11 ampl (drop over R2)

// GPIOs
//#define ADC_UNB_PIN  1                // ADC input from DAC output
//#define ADC_DAC_PIN  2                // ADC input from DAC output after Buffer Op-Amp
//#define ADC_AMP_PIN  3                // ADC input before R2
//#define ADC_SGN_PIN  4                // ADC input from x11 ampl (drop over R2)


// ADC Parameters
#define ADC_ATTEN    ADC_ATTEN_DB_0     // Maps ~0-3.3 V to full scale (attenuate for higher voltages, 0:k=100%, 2.5:k=75%, 6:k=50%, 12:k=25%)
#define ADC_WIDTH    ADC_BITWIDTH_13    // 13-bit resolution

// ===== Globl Vars =====
// Calibration factor for ADC
float calibrationFactor1;
float calibrationFactor2;
float calibrationFactor3;
float calibrationFactor4;

// ===== Functions =====

/*
Resolution: 13 Bit: 0-8191, or 12 bit: 0-4095
NEVER EXCEDE 3V3
Vdata = Vref/k * data/((2^bits)-1) where k is attenuation
*/

/**
 * @brief initialise ADC with proper configuration and calibration
 *        When to call: Once during initialization, after I2C init.
 * @return esp_err_t Error code
 */
esp_err_t adc_init(void);

/**
 * @brief Read ADC value from specified channel and return calibrated voltage
 * @param channel ADC channel to read from (ADC_UNB_CHN, ADC_DAC_CHN, etc.)
 * @param samples Number of samples to average (default: 64) (higher - slower)
 * @return float bit Voltage reading in volts
 */
float adc_read_voltage(adc1_channel_t channel, int samples);