/*  Header file for MS5837 Pressure/Temperature Sensor Driver
    Salinity Probe
    Zuhayr Loonat
    EEE4022S - 2025
*/

// ===== Includes =====
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ===== Defines =====
#define MS5837_ADDR         0x76        // MS5837 I2C address (can also be 0x77)
#define MS5837_RESET        0x1E        // Reset command
#define MS5837_ADC_READ     0x00        // ADC read command
#define MS5837_PROM_READ    0xA0        // PROM read command (base address)