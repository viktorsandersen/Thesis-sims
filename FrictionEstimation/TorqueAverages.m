%% Load Data and Compute Torque
data = readmatrix('data/robot_data_ur5_full_speed_wrench.csv');
actual_current = data(:, 44:49);
target_pos = data(:, 2:7);
actual_pos = data(:, 32:37);
target_vel = data(:, 8:13);
actual_vel = data(:, 38:43);
gear_ratio = 101;
torque_constants = [0.07695, 0.07695, 0.07695, 0.098322, 0.098322, 0.098322];

torque = zeros(size(actual_current));
for i = 1:6
    torque(:, i) = gear_ratio * torque_constants(i) * actual_current(:, i);
end

%% Segment the Data into Phases and Average Them
phase_length = 5000;               
total_samples = size(torque, 1);
num_phases = floor(total_samples / phase_length);

avg_phase_torque = zeros(phase_length, 6);
avg_phase_pos = zeros(phase_length, 6); % For storing the averaged positions
avg_phase_vel = zeros(phase_length, 6);
for joint = 1:6
    torque_reshaped = reshape(torque(1:num_phases*phase_length, joint), phase_length, num_phases);
    avg_phase_torque(:, joint) = mean(torque_reshaped, 2);
    pos_reshaped = reshape(actual_pos(1:num_phases*phase_length, joint), phase_length, num_phases);
    avg_phase_pos(:, joint) = mean(pos_reshaped, 2);
    vel_reshaped = reshape(actual_vel(1:num_phases*phase_length, joint), phase_length, num_phases);
    avg_phase_vel(:, joint) = mean(vel_reshaped, 2);
end

Fs = 1000;          
t_phase = (0:phase_length-1) / Fs;
csvwrite('AvgTorques.csv',avg_phase_torque)
% Create a figure for plotting
figure;
% Loop over each joint to create a subplot for each joint's torque phase
for joint = 1:6
    subplot(3, 2, joint);  % 3 rows, 2 columns of subplots, one for each joint
    
    % Loop through each phase and plot it on top of each other
    hold on;
    for phase = 1:num_phases
        start_idx = (phase - 1) * phase_length + 1;
        end_idx = phase * phase_length;
        
        % Plot each phase on the same axes (overlay phases)
        plot(1:phase_length, torque(start_idx:end_idx, joint));  % Plot phase torque
    end
    hold off;
    
    % Set title and labels
    xlabel('Samples in Phase');
    ylabel('Torque (Nm)');
    grid on;
    xlim([1000 2000]);
    % Add legend (optional, to identify the phases)
    legend('show');
end


%% Plot average torque
figure;
for joint = 1:6
    subplot(6,1,joint);
    plot(t_phase, avg_phase_torque(:, joint), 'b-', 'LineWidth', 1);
    %hold on;
    %plot(t_phase, avg_phase_torque_filtered(:, joint), 'g-', 'LineWidth', 1.5);
    %hold off;
    xlabel('Time (s)');
    ylabel(sprintf('Torque (Nm) - Joint %d', joint));
    legend('Averaged Phase');
    title(sprintf('Joint %d: Averaged Torque', joint));
end
%% Design and Apply the Butterworth Filter
%cutoff = 10;             % Cutoff frequency in Hz
%order = 4;               % Filter order

% Design the Butterworth filter (normalize cutoff with Fs/2)
%[b, a] = butter(order, cutoff/(Fs/2));

% Preallocate for the filtered signals
%avg_phase_torque_filtered = zeros(size(avg_phase_torque));
%for joint = 1:6
    % Apply zero-phase filtering to avoid phase distortion
    %avg_phase_torque_filtered(:, joint) = filtfilt(b, a, avg_phase_torque(:, joint));
%end

%% Plot the Results for Each Joint
%figure;
%for joint = 1:6
    %subplot(6,1,joint);
    %plot(t_phase, avg_phase_torque(:, joint), 'b-', 'LineWidth', 1);
    %hold on;
    %plot(t_phase, avg_phase_torque_filtered(:, joint), 'g-', 'LineWidth', 1.5);
    %hold off;
    %xlabel('Time (s)');
    %ylabel(sprintf('Torque (Nm) - Joint %d', joint));
    %legend('Averaged Phase', 'Butterworth Filtered');
    %title(sprintf('Joint %d: Averaged Phase Torque & Butterworth Filtering', joint));
%end
