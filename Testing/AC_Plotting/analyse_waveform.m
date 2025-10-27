% Filename
filename = 'ac_output.csv';

% Read file
data = readmatrix(filename);

% Extract the three lines
input_wave = data(1, :);
output_wave = data(2, :)*0.77;
time_scale = data(3, :);

% Calculate sampling frequency
dt = mean(diff(time_scale));  % Average time step
fs = 1/(dt*1e-6);  % Sampling frequency (convert microseconds to seconds)
N = length(time_scale);

% Create figure with subplots
figure('Position', [100, 100, 1200, 800]);

% Plot 1: Input and Output Waveforms
subplot(3, 2, 1);
plot(time_scale, input_wave, 'b-', 'LineWidth', 1.5);
hold on;
plot(time_scale, output_wave, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time');
ylabel('Amplitude');
title('Input and Output Waveforms');
legend('Input', 'Output');

% Plot 2: Individual Input Wave
subplot(3, 2, 2);
plot(time_scale, input_wave, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time');
ylabel('Amplitude');
title('Input Waveform');

% Plot 3: Individual Output Wave
subplot(3, 2, 3);
plot(time_scale, output_wave, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time');
ylabel('Amplitude');
title('Output Waveform');

% FFT Analysis for phase shift
input_fft = fft(input_wave);
output_fft = fft(output_wave);

% Frequency vector
freq = (0:N-1)*(fs/N);
freq = freq(1:floor(N/2));  % Only positive frequencies

% Magnitude spectrum
input_mag = abs(input_fft(1:floor(N/2)));
output_mag = abs(output_fft(1:floor(N/2)));

% Find dominant frequency (excluding DC component)
[~, idx] = max(input_mag(2:end));
idx = idx + 1;  % Adjust for skipping DC
dominant_freq = freq(idx);

% Phase spectrum
input_phase = angle(input_fft(1:floor(N/2)));
output_phase = angle(output_fft(1:floor(N/2)));

% Phase shift at dominant frequency (in radians and degrees)
phase_shift_rad = output_phase(idx) - input_phase(idx);
phase_shift_deg = rad2deg(phase_shift_rad);

% Wrap phase shift to [-180, 180] degrees
phase_shift_deg = wrapTo180(phase_shift_deg);
phase_shift_rad = deg2rad(phase_shift_deg);

% Calculate transfer function H(f) = Output/Input
transfer_function = output_fft ./ input_fft;
transfer_function = transfer_function(1:floor(N/2));

% Magnitude and phase of transfer function
H_magnitude = abs(transfer_function);
H_phase = angle(transfer_function);

% Amplitude ratio at dominant frequency
amplitude_ratio = output_mag(idx) / input_mag(idx);

% Plot 4: Magnitude Spectrum
subplot(3, 2, 4);
plot(freq, input_mag, 'b-', 'LineWidth', 1.5);
hold on;
plot(freq, output_mag, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('Magnitude Spectrum');
legend('Input', 'Output');
xlim([0, fs/2]);

% Plot 5: Transfer Function Magnitude (Bode Plot - Magnitude)
subplot(3, 2, 5);
plot(freq, 20*log10(H_magnitude), 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Transfer Function - Magnitude');
xlim([0, fs/2]);

% Plot 6: Transfer Function Phase (Bode Plot - Phase)
subplot(3, 2, 6);
plot(freq, rad2deg(H_phase), 'm-', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (Hz)');
ylabel('Phase (degrees)');
title('Transfer Function - Phase');
xlim([0, fs/2]);

% Display results in command window
fprintf('\n========== ANALYSIS RESULTS ==========\n');
fprintf('Sampling Frequency: %.2f Hz\n', fs);
fprintf('Number of Samples: %d\n', N);
fprintf('Dominant Frequency: %.2f Hz\n', dominant_freq);
fprintf('\n--- Amplitude Analysis ---\n');
fprintf('Input Amplitude (at dominant freq): %.4f\n', input_mag(idx));
fprintf('Output Amplitude (at dominant freq): %.4f\n', output_mag(idx));
fprintf('Amplitude Ratio (Output/Input): %.4f\n', amplitude_ratio);
fprintf('Gain (dB): %.2f dB\n', 20*log10(amplitude_ratio));
fprintf('\n--- Phase Analysis ---\n');
fprintf('Phase Shift: %.2f degrees (%.4f radians)\n', phase_shift_deg, phase_shift_rad);
fprintf('Time Delay: %.6f seconds\n', phase_shift_rad/(2*pi*dominant_freq));
fprintf('\n--- Transfer Function at Dominant Frequency ---\n');
fprintf('|H(f)|: %.4f\n', H_magnitude(idx));
fprintf('∠H(f): %.2f degrees\n', rad2deg(H_phase(idx)));
fprintf('======================================\n\n');

% Calculate RMS values
input_rms = rms(input_wave);
output_rms = rms(output_wave);

fprintf('--- RMS Values ---\n');
fprintf('Input RMS: %.4f\n', input_rms);
fprintf('Output RMS: %.4f\n', output_rms);
fprintf('RMS Ratio: %.4f\n', output_rms/input_rms);
fprintf('==================\n\n');

% Calculate peak-to-peak values
input_pp = max(input_wave) - min(input_wave);
output_pp = max(output_wave) - min(output_wave);

fprintf('--- Peak-to-Peak Values ---\n');
fprintf('Input P-P: %.4f\n', input_pp);
fprintf('Output P-P: %.4f\n', output_pp);
fprintf('P-P Ratio: %.4f\n', output_pp/input_pp);
fprintf('===========================\n\n');

% Print csv: input freq, input amp, impedance magnitude, impedance phase in deg
%fprintf('%.4f,%.4f,%.4f,%.4f\n', dominant_freq, input_mag(idx), H_magnitude(idx), rad2deg(H_phase(idx)));
fprintf('%.4f,%.4f,%.4f,%.4f\n', dominant_freq, max(abs(input_wave)), H_magnitude(idx), rad2deg(H_phase(idx)));

% Write to csv file
fileID = fopen('salinity_data.csv', 'a');
fprintf(fileID, '%.4f,%.4f,%.4f,%.4f\n', dominant_freq, max(abs(input_wave)), H_magnitude(idx), rad2deg(H_phase(idx)));
fclose(fileID);