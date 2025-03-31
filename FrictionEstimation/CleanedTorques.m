%% Read Data and Compute Torque
data = readmatrix('data/robot_data_speedj.csv');
data = data(1:10000, :);
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
%csvwrite('10kTorque.csv', torque)
%% Define Butterworth Filtering Parameters
torque = avg_phase_torque; % rm
torque(:,1) = torque(:,1) + (-7 + -0.6637);
torque(:,2) = torque(:,2) + (-30.9958 + 0.8315);
torque(:,3) = torque(:,3) + (-6.2407 + 0.0949);
torque(:,4) = torque(:,4) + (-2.8434 + 0.0019);
torque(:,5) = torque(:,5) + (2.1957 + 0.0534);
torque(:,6) = torque(:,6) + (-2.0350 + 0.0534);
Fs = 1000;       % Sampling frequency in Hz (adjust as needed)
cutoff = 5;     % Cutoff frequency in Hz
order = 2;       % Order of the Butterworth filter

% Design the Butterworth filter (normalized cutoff frequency)
[b, a] = butter(order, cutoff/(Fs/2));

% Create time vector
L = size(torque, 1);
t = (0:L-1) / Fs;

%% Apply Butterworth Filter and Plot Results for Each Joint
%figure;
x = torque;
x_filtered = filtfilt(b, a, x); 
%%
avg_phase_torque = zeros(10000,6)
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
    avg_phase_torque(:,joint) = x_filtered;
    %disp(avg_phase_torque(1,joint))
end
