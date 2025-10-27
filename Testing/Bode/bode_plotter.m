% Bode Plot Generator from CSV Data
% CSV should contain columns: freq, ampl, gain, phase shift

% Clear workspace
clear all;
close all;
clc;

% Read CSV file
filename = 'bode_data.csv';
data = readtable(filename);

% Extract data
freq = data.freq;           % Frequency (Hz)
gain = data.gain;           % Gain (linear, not dB)
phase = data.phase_shift;    % Phase shift (assuming degrees)

% Convert gain to dB
gain_dB = 20*log10(gain);

% Create Bode plot
figure('Position', [100, 100, 800, 600]);

% Magnitude plot
subplot(2,1,1);
plot(freq, gain_dB, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Bode Plot - Magnitude');

% Phase plot
subplot(2,1,2);
plot(freq, phase, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (Hz)');
ylabel('Phase (degrees)');
title('Bode Plot - Phase');

% Optional: Save figure
saveas(gcf, 'bode_plot.png');