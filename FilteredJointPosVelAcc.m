%% Load Data
data = readmatrix('data/robot_data_ur5_full_speed_wrench_first_5k.csv');
target_pos = data(:, 2:7);
actual_pos = data(:, 32:37);
target_vel = data(:, 8:13);
actual_vel = data(:, 38:43);  % adjusted to 6 columns

% Plot Target vs. Actual Positions for reference
figure;
for joint = 1:6
    subplot(6,1,joint);
    plot(target_pos(:, joint), 'b-', 'LineWidth', 1.5); hold on;
    plot(actual_pos(:, joint), 'r--', 'LineWidth', 1.5); hold off;
    xlabel('Sample Index');
    ylabel(sprintf('Joint %d Position', joint));
    title(sprintf('Joint %d: Target vs. Actual Position', joint));
    legend('Target', 'Actual');
end

%% FFT-Based Noise Removal and Frequency-Domain Differentiation for all joints
Fs = 10;         % Sampling frequency (Hz)
dt = 1/Fs;        % Sampling period
L = size(actual_pos, 1);    % Number of samples
T = 1/Fs;
t = (0:L-1)*T;    % Time vector

% Create frequency vector corresponding to the shifted FFT
if mod(L,2) == 0
    f = Fs * (-L/2:L/2-1) / L;
else
    f = Fs * (-(L-1)/2:(L-1)/2) / L;
end

cutoff = 0.1;  % Cutoff frequency in Hz

% Initialize arrays to store filtered signals
filtered_pos = zeros(size(actual_pos));
filtered_vel = zeros(size(actual_pos));
filtered_acc = zeros(size(actual_pos));
grad_vel = zeros(size(actual_pos));
grad_acc = zeros(size(actual_pos));

% Process each joint separately
for joint = 1:6
    x = actual_pos(:, joint);
    
    % Compute FFT of the position signal and shift zero-frequency to center
    Y = fft(x);
    Y_shifted = fftshift(Y);
    
    % Noise Removal: Zero out frequency components above the cutoff
    Y_shifted_filtered = Y_shifted;
    Y_shifted_filtered(abs(f) > cutoff) = 0;
    
    % Recover the filtered position signal in the time domain
    Y_filtered = ifftshift(Y_shifted_filtered);
    x_filtered = real(ifft(Y_filtered));
    filtered_pos(:, joint) = x_filtered;
    
    % Frequency-Domain Differentiation for Velocity
    V_shifted = 1i * 2 * pi * f' .* Y_shifted_filtered;
    V = ifftshift(V_shifted);
    x_dot_filtered = real(ifft(V));
    filtered_vel(:, joint) = x_dot_filtered;
    
    % Frequency-Domain Differentiation for Acceleration
    A_shifted = (1j * 2 * pi * f').^2 .* Y_shifted_filtered;
    A = ifftshift(A_shifted);
    x_ddot_filtered = real(ifft(A));
    filtered_acc(:, joint) = x_ddot_filtered;
    
    % Compute Time-Domain Gradients (optional for comparison)
    x_dot_gradient = gradient(x, dt);
    grad_vel(:, joint) = x_dot_gradient;
    x_ddot_gradient = gradient(x_dot_gradient, dt);
    grad_acc(:, joint) = x_ddot_gradient;
end

%% Plotting Results

% Plot Filtered vs. Actual Position for all joints
figure;
for joint = 1:6
    subplot(6,1,joint);
    plot(t, filtered_pos(:, joint), 'g', 'LineWidth', 1.5); hold on;
    plot(t, actual_pos(:, joint), 'r--', 'LineWidth', 1.5); hold off;
    xlabel('Time (s)');
    ylabel('Position');
    title(sprintf('Joint %d: Filtered vs. Actual Position', joint));
    legend('Filtered', 'Actual');
end

% Plot Velocity: Frequency-Domain Differentiated vs. Measured (actual and target) for all joints
figure;
for joint = 1:6
    subplot(6,1,joint);
    plot(t, filtered_vel(:, joint), 'g', 'LineWidth', 1.5); hold on;
    plot(t, actual_vel(:, joint), 'b--', 'LineWidth', 1.5); 
    plot(t, target_vel(:, joint), 'r', 'LineWidth', 1.5); hold off;
    xlabel('Time (s)');
    ylabel('Velocity');
    title(sprintf('Joint %d: Velocity Comparison', joint));
    legend('Filtered', 'Actual', 'Target');
end

% Plot Acceleration: Frequency-Domain Differentiated for all joints
figure;
for joint = 1:6
    subplot(6,1,joint);
    plot(t, filtered_acc(:, joint), 'g--', 'LineWidth', 1.5); hold on;
    % Optionally, to compare with time-domain gradient:
    % plot(t, grad_acc(:, joint), 'r--', 'LineWidth', 1.5);
    hold off;
    xlabel('Time (s)');
    ylabel('Acceleration');
    title(sprintf('Joint %d: Acceleration Comparison', joint));
    legend('Filtered Acceleration');
end
