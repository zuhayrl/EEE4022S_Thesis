/*  Header file for MCP4725 DAC Driver
    Salinity Probe
    Zuhayr Loonat
    EEE4022S - 2025
*/

// ===== Includes =====
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ===== Defines =====
#define MCP4725_ADDR 0x60               // MCP4725A0 default address
#define DAC_MAX_5V  4095                // 12-bit range
#define DAC_MAX_3V3 2200                // Limit for 3V3 ADC input out when powered at 5V

// ===== Functions =====

/*
    The DAC expects 3 bytes: 
    [ Command + PD bits ] [ DAC MSB ] [ DAC LSB ]
    For a fast DAC update: command = 0x40 (writes DAC register only, no EEPROM)
    12 Bit Value: 0-4095
    Power-down modes: Bits PD1/PD0 in the command byte let you put the output in high-impedance or pull-down states.
*/

/**
 * @brief Sets the output voltage of the MCP4725 DAC
 * @param value 12-bit value (0-4095) representing the desired output voltage
 * @param limit Boolean value to desgnate whether the DAC otput should be clamped to the ADC input
 * @return esp_err_t Error code from I²C write operation
 */
esp_err_t mcp4725_set_voltage(uint16_t value, bool limit);

/**
 * @brief Set DAC output voltage in volts (assuming 5V reference)
 * @param voltage Desired output voltage (0.0 to 5.0V, limited by DAC_MAX_3V3 for 3.3V output)
 * @param limit Boolean value to desgnate whether the DAC otput should be clamped to the ADC input
 * @return esp_err_t Error code from I²C write operation
 */
esp_err_t mcp4725_set_voltage_float(float voltage, bool limit);