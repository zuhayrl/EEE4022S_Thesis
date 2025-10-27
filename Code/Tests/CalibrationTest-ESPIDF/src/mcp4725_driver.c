/*  Code file for MCP4725 DAC Driver
    Salinity Probe
    Zuhayr Loonat
    EEE4022S - 2025
*/

// ===== Includes =====
#include "mcp4725_driver.h"
#include "i2c_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdbool.h>

// ===== Functions =====

/*
    The DAC expects 3 bytes: 
    [ Command + PD bits ] [ DAC MSB ] [ DAC LSB ]
    For a fast DAC update: command = 0x40 (writes DAC register only, no EEPROM)
    12 Bit Value: 0-4095
    Power-down modes: Bits PD1/PD0 in the command byte let you put the output in high-impedance or pull-down states.
*/

/**
 * @brief Sets the output voltage of the MCP4725 DAC.
 * @param value 12-bit value (0-4095) representing the desired output voltage.
 * @param limit Boolean value to desgnate whether the DAC otput should be clamped to the ADC input 
 * @return esp_err_t Error code from I²C write operation.
 */
esp_err_t mcp4725_set_voltage(uint16_t value, bool limit){
    
    // Check if user inputted voltage is higher than allowed voltage
    if (value > DAC_MAX_5V) {
        value = DAC_MAX_5V; // Clamp to maximum value
    }

    // Limit to 3.3V output if specified
    if (limit && (value > DAC_MAX_3V3)) {
        printf("MCP4725: Warning - voltage %.2fV limited to 3.3V output\n");
        value = DAC_MAX_3V3; // Clamp to limit
    }
    
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

/**
 * @brief Set DAC output voltage in volts (assuming 5V reference)
 * @param voltage Desired output voltage (0.0 to 5.0V, limited by DAC_MAX_3V3 for 3.3V output)
 * @param limit Boolean value to desgnate whether the DAC otput should be clamped to the ADC input
 * @return esp_err_t Error code from I²C write operation
 */
esp_err_t mcp4725_set_voltage_float(float voltage, bool limit){
    if (voltage < 0.0f) voltage = 0.0f;
    if (voltage > 5.0f) voltage = 5.0f;
    
    // Convert voltage to 12-bit value (0-4095)
    // Assuming 5V reference: value = (voltage / 5.0) * 4095
    uint16_t dac_value = (uint16_t)((voltage / 5.0f) * DAC_MAX_5V);
    
    // Limit to 3.3V output if specified
    if (limit && (dac_value > DAC_MAX_3V3)) {
        printf("MCP4725: Warning - voltage %.2fV limited to 3.3V output\n", voltage);
        dac_value = DAC_MAX_3V3;
    }
    
    return mcp4725_set_voltage(dac_value, limit);
}