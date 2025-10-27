/*  Code file for I2C Driver
    Salinity Probe
    Zuhayr Loonat
    EEE4022S - 2025
*/

// ===== Includes =====
#include "i2c_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
// ===== Defines =====

// ===== Global Vars =====
static bool i2c_initialised = false;

// ===== Functions =====

/**
 * @brief initialise I2C master mode
 * @return esp_err_t Error code
 */
esp_err_t i2c_master_init(void){
    
    // Check if I2C is initialised
    if (i2c_initialised) {
        return ESP_OK; // Already initialised
    }

    // Configure I2C parameters
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,                // Set as I2C master
        .sda_io_num = SDA_PIN,                  // SDA pin number
        .scl_io_num = SCL_PIN,                  // SCL pin number
        .sda_pullup_en = GPIO_PULLUP_ENABLE,    // Enable pull-up on SDA
        .scl_pullup_en = GPIO_PULLUP_ENABLE,    // Enable pull-up on SCL
        .master.clk_speed = I2C_FREQ,           // Set I2C clock frequency (Hz)
    };

    // Apply I2C configuration to the specified port
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    
    // Install I2C driver for the port
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
    
    i2c_initialised = true;
    printf("I2C: initialised at %d Hz\n", I2C_FREQ);
    
    return ESP_OK;
}

/**
 * @brief Deinitialise I2C to free resources
 * @return esp_err_t Error code
 */
esp_err_t i2c_master_deinit(void){
    if (!i2c_initialised) {
        return ESP_OK;
    }
    
    esp_err_t ret = i2c_driver_delete(I2C_PORT);
    if (ret == ESP_OK) {
        i2c_initialised = false;
        printf("I2C: Deinitialised\n");
    }
    
    return ret;
}