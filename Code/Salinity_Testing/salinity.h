#ifndef SALINITY_H
#define SALINITY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calculate resistance from voltage divider circuit
 * 
 * @param v_input Input voltage applied to the voltage divider (V)
 * @param v_probe Voltage measured across the probe (V)
 * @param r1 Known resistor value in series with probe (Ohms)
 * @return Probe resistance (Ohms), or -1.0 on error
 */
float calculate_resistance(float v_input, float v_probe, float r1);

/**
 * @brief Calculate conductivity from resistance and probe geometry
 * 
 * @param resistance Measured resistance across probe (Ohms)
 * @param distance Distance between electrode plates (m)
 * @param area Area of electrode plates (m^2)
 * @return Conductivity (S/m - Siemens per meter), or -1.0 on error
 */
float calculate_conductivity(float resistance, float distance, float area);

/**
 * @brief Calculate conductivity ratio Rt from temperature
 * Helper function for salinity calculation
 * 
 * @param temp Temperature (C)
 * @return Rt value
 */
float calculate_rt(float temp);

/**
 * @brief Calculate Rp (pressure correction factor)
 * For surface measurements (pressure = 0), this returns 1.0
 * 
 * @param R Conductivity ratio
 * @param temp Temperature (C)
 * @param pressure Pressure (dbar) - use 0 for surface
 * @return Rp value
 */
float calculate_rp(float R, float temp, float pressure);

/**
 * @brief Calculate salinity from conductivity ratio using UNESCO PSS-78
 * 
 * @param R Conductivity ratio (sample_conductivity / standard_conductivity at same temp)
 * @param temp Temperature (C)
 * @param pressure Pressure (dbar) - use 0 for surface measurements
 * @return Practical Salinity (PSU)
 */
float calculate_salinity_from_ratio(float R, float temp, float pressure);

/**
 * @brief Calculate salinity from test solution conductivity
 * 
 * @param conductivity_sample Conductivity of sample solution (S/m)
 * @param conductivity_standard Conductivity of standard solution (S/m)
 * @param temp_sample Temperature of sample (C)
 * @param temp_standard Temperature of standard (C) - typically 15C
 * @param pressure Pressure (dbar) - use 0 for surface measurements
 * @return Practical Salinity (PSU)
 */
float calculate_salinity(float conductivity_sample, float conductivity_standard,
                        float temp_sample, float temp_standard, float pressure);

/**
 * @brief Complete pipeline: voltage to salinity
 * 
 * @param v_input Input voltage (V)
 * @param v_probe Measured voltage across probe (V)
 * @param r1 Series resistor value (Ohms)
 * @param distance Electrode spacing (m)
 * @param area Electrode area (m^2)
 * @param conductivity_standard Standard solution conductivity (S/m)
 * @param temp_sample Sample temperature (C)
 * @param temp_standard Standard temperature (C)
 * @return Practical Salinity (PSU), or negative value on error
 */
float voltage_to_salinity(float v_input, float v_probe, float r1,
                          float distance, float area,
                          float conductivity_standard,
                          float temp_sample, float temp_standard);

#ifdef __cplusplus
}
#endif

#endif // SALINITY_H