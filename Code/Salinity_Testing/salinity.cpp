// Area = 0.0004m^2 Distnace = 0.01m

#include "salinity.h"
#include <math.h>

float calculate_resistance(float v_input, float v_probe, float r1) {
    if (v_input <= v_probe) {
        return -1.0f; // Error: invalid voltage readings
    }
    return r1 * v_probe / (v_input - v_probe);
}

float calculate_conductivity(float resistance, float distance, float area) {
    if (resistance <= 0 || area <= 0) {
        return -1.0f; // Error: invalid parameters
    }
    return distance / (resistance * area);
}

float calculate_rt(float temp) {
    // Coefficients for Rt polynomial
    const float c0 = 0.6766097;
    const float c1 = 2.00564e-2;
    const float c2 = 1.104259e-4;
    const float c3 = -6.9698e-7;
    const float c4 = 1.0031e-9;
    
    return c0 + c1*temp + c2*temp*temp + c3*temp*temp*temp + c4*temp*temp*temp*temp;
}

float calculate_rp(float R, float temp, float pressure) {
    if (pressure == 0) {
        return 1.0f;
    }
    
    // Coefficients for pressure correction
    const float e1 = 2.070e-5;
    const float e2 = -6.370e-10;
    const float e3 = 3.989e-15;
    
    const float d1 = 3.426e-2;
    const float d2 = 4.464e-4;
    const float d3 = 4.215e-1;
    const float d4 = -3.107e-3;
    
    float Rp_num = 1 + (pressure * (e1 + e2*pressure + e3*pressure*pressure)) / 
                   (1 + d1*temp + d2*temp*temp + (d3 + d4*temp)*R);
    
    return Rp_num;
}

float calculate_salinity_from_ratio(float R, float temp, float pressure) {
    if (R <= 0) {
        return 0.0f; // No conductivity means no salinity
    }
    
    // Calculate Rt (temperature correction)
    float Rt = calculate_rt(temp);
    
    // Calculate Rp (pressure correction)
    float Rp = calculate_rp(R, temp, pressure);
    
    // Calculate RT (temperature and pressure corrected ratio)
    float RT = R / (Rt * Rp);
    
    // Coefficients for salinity calculation
    const float a0 = 0.0080;
    const float a1 = -0.1692;
    const float a2 = 25.3851;
    const float a3 = 14.0941;
    const float a4 = -7.0261;
    const float a5 = 2.7081;
    
    const float b0 = 0.0005;
    const float b1 = -0.0056;
    const float b2 = -0.0066;
    const float b3 = -0.0375;
    const float b4 = 0.0636;
    const float b5 = -0.0144;
    
    const float k = 0.0162;
    
    // Calculate sqrt(RT)
    float sqrt_RT = sqrtf(RT);
    
    // Calculate deltaS (salinity difference from 35 PSU)
    float deltaS = (temp - 15.0f) / (1.0f + k * (temp - 15.0f)) *
                   (b0 + b1*sqrt_RT + b2*RT + b3*RT*sqrt_RT + 
                    b4*RT*RT + b5*RT*RT*sqrt_RT);
    
    // Calculate salinity
    float salinity = a0 + a1*sqrt_RT + a2*RT + a3*RT*sqrt_RT + 
                     a4*RT*RT + a5*RT*RT*sqrt_RT + deltaS;
    
    return salinity;
}

float calculate_salinity(float conductivity_sample, float conductivity_standard,
                        float temp_sample, float temp_standard, float pressure) {
    // Calculate conductivity ratio at measurement temperature
    float R = conductivity_sample / conductivity_standard;
    
    // Need to correct for temperature difference between sample and standard
    // Calculate what the ratio would be if both were at the same temperature
    float Rt_sample = calculate_rt(temp_sample);
    float Rt_standard = calculate_rt(temp_standard);
    
    // Correct the ratio to account for different temperatures
    float R_corrected = R * (Rt_standard / Rt_sample);
    
    // Calculate salinity using the corrected ratio at sample temperature
    return calculate_salinity_from_ratio(R_corrected, temp_sample, pressure);
}

float voltage_to_salinity(float v_input, float v_probe, float r1,
                          float distance, float area,
                          float conductivity_standard,
                          float temp_sample, float temp_standard) {
    // Step 1: Calculate resistance
    float resistance = calculate_resistance(v_input, v_probe, r1);
    if (resistance < 0) return -1.0f;
    
    // Step 2: Calculate conductivity
    float conductivity = calculate_conductivity(resistance, distance, area);
    if (conductivity < 0) return -2.0f;
    
    // Step 3: Calculate salinity
    return calculate_salinity(conductivity, conductivity_standard,
                            temp_sample, temp_standard, 0.0f);
}