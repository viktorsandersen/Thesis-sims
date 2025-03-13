%% Plot data
% Pos
data = readmatrix('data/robot_data_ur5_full_speed_wrench_first_5k.csv');
target_pos = data(:, 2:7);
actual_pos = data(:, 32:37);

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

%% Vel
target_vel = data(:, 8:13);
actual_vel = data(:, 38:44);

figure;
for joint = 1:6
    subplot(6,1,joint);
    plot(target_vel(:, joint), 'b-', 'LineWidth', 1.5); hold on;
    plot(actual_vel(:, joint), 'r--', 'LineWidth', 1.5); hold off;
    xlabel('Sample Index');
    ylabel(sprintf('Joint %d Velocity', joint));
    title(sprintf('Joint %d: Target vs. Actual Velocity', joint));
    legend('Target', 'Actual');
end

%% Acc
target_acc = data(:, 14:19);
%actual_acc = data(:, 45:50); %This doesn't exist

figure;
for joint = 1:6
    subplot(6,1,joint);
    plot(target_acc(:, joint), 'b-', 'LineWidth', 1.5);
    xlabel('Sample Index');
    ylabel(sprintf('Joint %d Acceleration', joint));
    title(sprintf('Joint %d: Target vs. Actual Acceleration', joint));
    legend('Target');
end
%% Torque
actual_current = data(:, 44:49);

gear_ratio = 110;
torque_constants = [0.07695, 0.07695, 0.098322, 0.07695, 0.098322, 0.098322];
% torque = gear_ratio * torque_constant * current
torque = zeros(size(actual_current));
for i = 1:6
    torque(:, i) = gear_ratio * torque_constants(i) * actual_current(:, i);
end

figure;
for i = 1:6
    subplot(6, 1, i);
    plot(torque(:, i));
    xlabel('Sample Index');
    ylabel(sprintf('Torque (Nm) - Joint %d', i-1));
    title(sprintf('Calculated Torque from Actual Current %d', i-1));
end

%% FFT
t = 0.002:0.002:10;
Fs = 500;           % Sampling freq
dt = 1/Fs           % ts
x = torque(:,1);
N = length(x);       % Number of samples
X = fft(x);
f = (0:N-1)*(Fs/N);
cutoff = 400;
X_filtered = X;
X_filtered(f > cutoff) = 0;
x_filtered = real(ifft(X_filtered));
x_dot = gradient(x_filtered, dt);    % Velocity
x_ddot = gradient(x_dot, dt);        % Acceleration
Y = [x_ddot, x_dot, x_filtered];
disp(size(t));
disp(size(x));
disp(size(x_filtered));
figure;
plot(t, x, 'b-', t, x_filtered, 'r-');
xlabel('Time (s)');
ylabel('Position');
legend('Original', 'Filtered');
title('Signal Filtering via FFT');

%subplot(3,1,2);
%plot(t, x_dot, 'k-');
%xlabel('Time (s)');
%ylabel('Velocity');
%title('Computed Velocity');

%subplot(3,1,3);
%plot(t, x_ddot, 'm-');
%xlabel('Time (s)');
%ylabel('Acceleration');
%title('Computed Acceleration');

%% Single-Sided Amplitude Spectrum Plot
% Compute the two-sided spectrum P2 and then derive the single-sided spectrum P1.
P2 = abs(X/N);             % Two-sided spectrum
P1 = P2(1:N/2+1);   % Single-sided spectrum
P1(2:end-1) = 2*P1(2:end-1); % Compensate for dropping second half of FFT

% Define frequency domain f for single-sided spectrum
f_single = Fs*(0:(N/2))/N;

figure;
plot(f_single, P1, 'LineWidth', 3);
title('Single-Sided Amplitude Spectrum of X(t)');
xlabel('f (Hz)');
ylabel('|P1(f)|');

%% Approach 2: Time-Domain Butterworth Filter

cutoff_butter = 10;
order = 4;  %order

[b, a] = butter(order, cutoff_butter/(Fs/2));

% Apply zero-phase - avoid phase distortion
x_filtered_butter = filtfilt(b, a, x);

figure;

% Subplot 1: Original vs. FFT-filtered signal
subplot(3,1,1);
plot(t, x, 'b-', t, x_filtered, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Torque (Nm)');
legend('Original', 'FFT Filtered');
title('Torque Signal Filtering via FFT');

% Subplot 2: Original vs. Butterworth-filtered signal
subplot(3,1,2);
plot(t, x, 'b-', t, x_filtered_butter, 'g-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Torque (Nm)');
legend('Original', 'Butterworth Filtered');
title('Torque Signal Filtering via Butterworth');

% Subplot 3: FFT-filtered vs. Butterworth-filtered signal
subplot(3,1,3);
plot(t, x_filtered, 'r-', t, x_filtered_butter, 'g-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Torque (Nm)');
legend('FFT Filtered', 'Butterworth Filtered');
title('Comparison: FFT vs. Butterworth Filtering');


