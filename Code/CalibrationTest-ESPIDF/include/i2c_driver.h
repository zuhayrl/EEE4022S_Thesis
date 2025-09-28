/*  Header file for I2C Driver
    Salinity Probe
    Zuhayr Loonat
    EEE4022S - 2025
*/

// ===== Includes =====
#include "driver/i2c.h"
#include "esp_err.h"

// ===== Defines =====
#define I2C_PORT     I2C_NUM_0          // I2C Port Number
#define I2C_FREQ     100000             // 100 kHz
#define SDA_PIN      8                  // SDA GPIO PIN 
#define SCL_PIN      9                  // SCL GPIO PIN

// ===== Functions =====

/**
 * @brief initialise I2C master mode
 * @return esp_err_t Error code
 */
esp_err_t i2c_master_init(void);

/**
 * @brief Deinitialise I²C and free resources
 * @return esp_err_t Error code
 */
esp_err_t i2c_master_deinit(void);