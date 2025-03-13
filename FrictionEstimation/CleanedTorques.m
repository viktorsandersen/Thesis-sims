%% Read Data and Compute Torque
data = readmatrix('data/robot_data_ur5_full_speed_wrench_first_5k.csv');
% Extract joint positions (for reference) and currents:
pos = data(:, 2:7);
actual_current = data(:, 44:49);

% Define parameters for torque calculation
gear_ratio = 110;
torque_constants = [0.07695, 0.07695, 0.098322, 0.07695, 0.098322, 0.098322];

% Compute torque for each joint
torque = zeros(size(actual_current));
for i = 1:6
    torque(:, i) = gear_ratio * torque_constants(i) * actual_current(:, i);
end

%% Define Butterworth Filtering Parameters
Fs = 1000;       % Sampling frequency in Hz (adjust as needed)
cutoff = 10;     % Cutoff frequency in Hz
order = 4;       % Order of the Butterworth filter

% Design the Butterworth filter (normalized cutoff frequency)
[b, a] = butter(order, cutoff/(Fs/2));

% Create time vector
L = size(torque, 1);
t = (0:L-1) / Fs;

%% Apply Butterworth Filter and Plot Results for Each Joint
figure;
x = torque;
x_filtered = filtfilt(b, a, x); 
%%
for joint = 1:6
    x = torque(:, joint);    
    % Apply zero-phase filtering to avoid phase distortion
    x_filtered = filtfilt(b, a, x);
    
    subplot(6,1,joint);
    plot(t, x, 'b-', 'LineWidth', 1); hold on;
    plot(t, x_filtered, 'g-', 'LineWidth', 1.5); hold off;
    xlabel('Time (s)');
    ylabel(sprintf('Torque (Nm) - Joint %d', joint));
    legend('Original', 'Butterworth Filtered');
    title(sprintf('Joint %d: Butterworth Filtering', joint));
end
