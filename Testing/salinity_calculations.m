%% Salinity Calculation Functions
% UNESCO Practical Salinity Scale 1978 (PSS-78) implementation
% For conductivity probe measurements at surface pressure (0 dbar)

%% Main calculation pipeline
function salinity = voltage_to_salinity(v_input, v_probe, r1, distance, area, conductivity_standard, temp_sample, temp_standard)
    % Complete pipeline: voltage to salinity
    %
    % Inputs:
    %   v_input - Input voltage (V)
    %   v_probe - Measured voltage across probe (V)
    %   r1 - Series resistor value (Ohms)
    %   distance - Electrode spacing (m)
    %   area - Electrode area (m²)
    %   conductivity_standard - Standard solution conductivity (S/m)
    %   temp_sample - Sample temperature (°C)
    %   temp_standard - Standard temperature (°C), typically 15
    %
    % Output:
    %   salinity - Practical Salinity (PSU)
    
    % Step 1: Calculate resistance
    resistance = calculate_resistance(v_input, v_probe, r1);
    if resistance < 0
        error('Invalid voltage readings');
    end
    
    % Step 2: Calculate conductivity
    conductivity = calculate_conductivity(resistance, distance, area);
    if conductivity < 0
        error('Invalid resistance or geometry parameters');
    end
    
    % Step 3: Calculate salinity
    salinity = calculate_salinity(conductivity, conductivity_standard, temp_sample, temp_standard, 0);
end

%% Individual calculation functions

function resistance = calculate_resistance(v_input, v_probe, r1)
    % Calculate resistance from voltage divider circuit
    %
    % Inputs:
    %   v_input - Input voltage applied to voltage divider (V)
    %   v_probe - Voltage measured across probe (V)
    %   r1 - Known resistor value in series with probe (Ohms)
    %
    % Output:
    %   resistance - Probe resistance (Ohms)
    %
    % Circuit: V_input -- R1 -- [V_probe] -- R_probe -- GND
    % Formula: R_probe = R1 * V_probe / (V_input - V_probe)
    
    if v_input <= v_probe
        resistance = -1;
        warning('Invalid voltage readings: v_input must be greater than v_probe');
        return;
    end
    
    resistance = r1 * v_probe / (v_input - v_probe);
end

function conductivity = calculate_conductivity(resistance, distance, area)
    % Calculate conductivity from resistance and probe geometry
    %
    % Inputs:
    %   resistance - Measured resistance across probe (Ohms)
    %   distance - Distance between electrode plates (m)
    %   area - Area of electrode plates (m²)
    %
    % Output:
    %   conductivity - Conductivity (S/m)
    %
    % Formula: Conductivity = distance / (resistance * area)
    
    if resistance <= 0 || area <= 0
        conductivity = -1;
        warning('Invalid parameters: resistance and area must be positive');
        return;
    end
    
    conductivity = distance / (resistance * area);
end

function Rt = calculate_rt(temp)
    % Calculate conductivity ratio Rt from temperature
    % Helper function for salinity calculation
    %
    % Input:
    %   temp - Temperature (°C)
    %
    % Output:
    %   Rt - Temperature correction factor
    
    % Coefficients for Rt polynomial
    c0 = 0.6766097;
    c1 = 2.00564e-2;
    c2 = 1.104259e-4;
    c3 = -6.9698e-7;
    c4 = 1.0031e-9;
    
    Rt = c0 + c1*temp + c2*temp^2 + c3*temp^3 + c4*temp^4;
end

function Rp = calculate_rp(R, temp, pressure)
    % Calculate Rp (pressure correction factor)
    % For surface measurements (pressure = 0), returns 1.0
    %
    % Inputs:
    %   R - Conductivity ratio
    %   temp - Temperature (°C)
    %   pressure - Pressure (dbar), use 0 for surface
    %
    % Output:
    %   Rp - Pressure correction factor
    
    if pressure == 0
        Rp = 1.0;
        return;
    end
    
    % Coefficients for pressure correction
    e1 = 2.070e-5;
    e2 = -6.370e-10;
    e3 = 3.989e-15;
    
    d1 = 3.426e-2;
    d2 = 4.464e-4;
    d3 = 4.215e-1;
    d4 = -3.107e-3;
    
    Rp = 1 + (pressure * (e1 + e2*pressure + e3*pressure^2)) / ...
         (1 + d1*temp + d2*temp^2 + (d3 + d4*temp)*R);
end

function salinity = calculate_salinity_from_ratio(R, temp, pressure)
    % Calculate salinity from conductivity ratio using UNESCO PSS-78
    %
    % Inputs:
    %   R - Conductivity ratio (sample/standard at same temp)
    %   temp - Temperature (°C)
    %   pressure - Pressure (dbar), use 0 for surface
    %
    % Output:
    %   salinity - Practical Salinity (PSU)
    %
    % Valid for: 2 ≤ S ≤ 42 PSU, -2 ≤ T ≤ 35°C, 0 ≤ P ≤ 10000 dbar
    
    if R <= 0
        salinity = 0;
        return;
    end
    
    % Calculate Rt (temperature correction)
    Rt = calculate_rt(temp);
    
    % Calculate Rp (pressure correction)
    Rp = calculate_rp(R, temp, pressure);
    
    % Calculate RT (temperature and pressure corrected ratio)
    RT = R / (Rt * Rp);
    
    % Coefficients for salinity calculation
    a0 = 0.0080;
    a1 = -0.1692;
    a2 = 25.3851;
    a3 = 14.0941;
    a4 = -7.0261;
    a5 = 2.7081;
    
    b0 = 0.0005;
    b1 = -0.0056;
    b2 = -0.0066;
    b3 = -0.0375;
    b4 = 0.0636;
    b5 = -0.0144;
    
    k = 0.0162;
    
    % Calculate sqrt(RT)
    sqrt_RT = sqrt(RT);
    
    % Calculate deltaS (salinity difference from 35 PSU)
    deltaS = (temp - 15.0) / (1.0 + k * (temp - 15.0)) * ...
             (b0 + b1*sqrt_RT + b2*RT + b3*RT*sqrt_RT + ...
              b4*RT^2 + b5*RT^2*sqrt_RT);
    
    % Calculate salinity
    salinity = a0 + a1*sqrt_RT + a2*RT + a3*RT*sqrt_RT + ...
               a4*RT^2 + a5*RT^2*sqrt_RT + deltaS;
end

function salinity = calculate_salinity(conductivity_sample, conductivity_standard, temp_sample, temp_standard, pressure)
    % Calculate salinity from test solution conductivity
    %
    % Inputs:
    %   conductivity_sample - Conductivity of sample solution (S/m)
    %   conductivity_standard - Conductivity of standard solution (S/m)
    %   temp_sample - Temperature of sample (°C)
    %   temp_standard - Temperature of standard (°C), typically 15
    %   pressure - Pressure (dbar), use 0 for surface
    %
    % Output:
    %   salinity - Practical Salinity (PSU)
    %
    % Standard solution is typically 35 PSU at 15°C
    
    % Calculate conductivity ratio at measurement temperature
    R = conductivity_sample / conductivity_standard;
    
    % Correct for temperature difference between sample and standard
    Rt_sample = calculate_rt(temp_sample);
    Rt_standard = calculate_rt(temp_standard);
    
    % Correct the ratio to account for different temperatures
    R_corrected = R * (Rt_standard / Rt_sample);
    
    % Calculate salinity using the corrected ratio at sample temperature
    salinity = calculate_salinity_from_ratio(R_corrected, temp_sample, pressure);
end

%% Example usage
% Uncomment to test the functions

% Probe parameters
V_INPUT = 1.65;              % ESP32 voltage (V)
R1 = 100;                  % 1k ohm series resistor
d = 0.01;      % 1cm between plates (m)
A = 0.0004;        % 2cm² plates (m²)
 
% Standard solution (35 PSU at 15°C) - measure this value
STANDARD_CONDUCTIVITY = 5.0; % S/m (example value, measure yours!)
STANDARD_TEMP = 15.0;        % °C
 
% Sample measurement
v_probe = 1.5;               % Measured voltage (V)
sample_temp = 25.0;          % Sample temperature (°C)
 
% Or step-by-step:
resistance = calculate_resistance(V_INPUT, v_probe, R1);
fprintf('Resistance: %.2f Ohms\n', resistance);
 
conductivity = calculate_conductivity(resistance, d, A);
fprintf('Conductivity: %.4f S/m\n', conductivity);
% 
% salinity = calculate_salinity(conductivity, STANDARD_CONDUCTIVITY, ...
%                               sample_temp, STANDARD_TEMP, 0);
% fprintf('Salinity: %.2f PSU\n', salinity);

% Calculate salinity
%salinity = voltage_to_salinity(V_INPUT, v_probe, R1, ...
%                                PROBE_DISTANCE, PROBE_AREA, ...
%                                STANDARD_CONDUCTIVITY, ...
%                                sample_temp, STANDARD_TEMP);
% 
%fprintf('Salinity: %.2f PSU\n', salinity);
