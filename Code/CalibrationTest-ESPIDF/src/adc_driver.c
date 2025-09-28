/*  Code file for ADC Driver
    Salinity Probe
    Zuhayr Loonat
    EEE4022S - 2025
*/

// ===== Includes =====
#include "adc_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

// ===== Global Vars =====
static esp_adc_cal_characteristics_t *adc_chars;

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
esp_err_t adc_init(void){

    // Configure ADC1 (Channels 0-3) for specified width and attenuation
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_UNB_CHN, ADC_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_DAC_CHN, ADC_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_AMP_CHN, ADC_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_SGN_CHN, ADC_ATTEN));

    // Characterize ADC for voltage calculation
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    if (adc_chars == NULL) {
        return ESP_ERR_NO_MEM;
    }
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, 1100, adc_chars);

    // Print calibration information
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("ADC: Characterized using eFuse Vref\n");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("ADC: Characterized using Two Point Value\n");
    }
    else {
        printf("ADC: Characterized using Default Vref\n");
    }

    return ESP_OK;
}

/**
 * @brief Read ADC value from specified channel and return calibrated voltage
 * @param channel ADC channel to read from (ADC_UNB_CHN, ADC_DAC_CHN, etc.)
 * @param samples Number of samples to average (default: 64) (higher - slower)
 * @return float bit Voltage reading in volts
 */
float adc_read_voltage(adc1_channel_t channel, int samples){
    
    // Check if ADC is initialised
    if (adc_chars == NULL) {
        printf("ADC: Not initialised!\n");
        return 0.0;
    }
    
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

    // Check if ADC is initialised
    if (adc_chars == NULL) {
        printf("ADC: Not initialised!\n");
        return 0.0;
    }
    
    if (samples <= 0) samples = 64; // Default sampling
    uint32_t adc_reading = 0;

    // Take multiple samples for better accuracy
    for (int i = 0; i < samples; i++) {
        adc_reading += adc1_get_raw(channel);
    }

    return adc_reading / samples;
}
