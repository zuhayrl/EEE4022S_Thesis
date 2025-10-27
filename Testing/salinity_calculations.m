%% Salinity Calculation Functions
% For conductivity probe measurements at surface pressure (0 dbar)

%% Main calculation pipeline
function salinity = voltage_to_salinity(v_input, v_probe, r1, distance, area, conductivity_standard, temp_sample, temp_standard)
    
    % Calculate resistance
    resistance = calculate_resistance(v_input, v_probe, r1);
    if resistance < 0
        error('Invalid voltage readings');
    end
    
    % Calculate conductivity
    conductivity = calculate_conductivity(resistance, distance, area);
    if conductivity < 0
        error('Invalid resistance or geometry parameters');
    end
    
    % Calculate salinity
    salinity = calculate_salinity(conductivity, conductivity_standard, temp_sample, temp_standard, 0);
end

%% Individual calculation functions
% Resistance
function resistance = calculate_resistance(v_input, v_probe, r1)
    
    if v_input <= v_probe
        resistance = -1;
        warning('Invalid voltage readings: v_input must be greater than v_probe');
        return;
    end
    
    resistance = r1 * v_probe / (v_input - v_probe);
end
% Conductivity
function conductivity = calculate_conductivity(resistance, distance, area)
    
    if resistance <= 0 || area <= 0
        conductivity = -1;
        warning('Invalid parameters: resistance and area must be positive');
        return;
    end
    
    conductivity = distance / (resistance * area);
end

% Temp ratio
function Rt = calculate_rt(temp)
    
    % Coefficients
    c0 = 0.6766097;
    c1 = 2.00564e-2;
    c2 = 1.104259e-4;
    c3 = -6.9698e-7;
    c4 = 1.0031e-9;
    
    Rt = c0 + c1*temp + c2*temp^2 + c3*temp^3 + c4*temp^4;
end

% Pressure correction factor
function Rp = calculate_rp(R, temp, pressure)
    
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

% Salinity from ratios 
function salinity = calculate_salinity_from_ratio(R, temp, pressure)
    
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

% Salinity from cond, temp, pres
function salinity = calculate_salinity(conductivity_sample, conductivity_standard, temp_sample, temp_standard, pressure)

    
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

%% Usage

% Probe parameters
V_INPUT = 1.4;             
R1 = 100;               
d = 0.01;      % 1cm between plates (m)
A = 0.0004;        % 2cm^2 plates (m^2)
 
% Standard solution (35 PSU at 15C) - measure this value
STANDARD_CONDUCTIVITY = 3.53; % S/m 
STANDARD_TEMP = 15.0;        % C
 
% Sample measurement
v_probe = 0.143;               % Measured voltage (V)
sample_temp = 15;          % Sample temperature (C)
 
% Or step-by-step:
resistance = calculate_resistance(V_INPUT, v_probe, R1);
fprintf('Resistance: %.2f Ohms\n', resistance);
 
conductivity = calculate_conductivity(resistance, d, A);
fprintf('Conductivity: %.4f S/m\n', conductivity);

salinity = calculate_salinity(conductivity, STANDARD_CONDUCTIVITY, ...
                               sample_temp, STANDARD_TEMP, 0);
fprintf('Salinity: %.2f PSU\n', salinity);

% Calculate salinity
%salinity = voltage_to_salinity(V_INPUT, v_probe, R1, ...
%                                PROBE_DISTANCE, PROBE_AREA, ...
%                                STANDARD_CONDUCTIVITY, ...
%                                sample_temp, STANDARD_TEMP);
% 
%fprintf('Salinity: %.2f PSU\n', salinity);
