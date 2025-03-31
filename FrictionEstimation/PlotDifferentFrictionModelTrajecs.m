data = readmatrix('robot_data4.5,9.3.csv');
%actual_current = data(:, 44:49);
target_pos = data(2000:2800, 2:7);
actual_pos = data(2000:2800, 32:37);
target_vel = data(2000:2800, 8:13);
actual_vel = data(2000:2800, 38:43);

figure;
for joint = 1:6
    subplot(6,1,joint);
    plot(target_pos(:, joint), 'b-', 'LineWidth', 1.5); hold on;
    plot(actual_pos(:, joint), 'r--', 'LineWidth', 1.5); %hold off;
    xlabel('Sample Index');
    ylabel(sprintf('Joint %d Position', joint));
    title(sprintf('4.5, 9.3'));
    legend('Actual_vel','actual pos');
end
disp('4.5,9.3')
[rise_time, settling_time, overshoot, steady_state_error] = compute_metrics(target_pos(:, 1), actual_pos(:, 1))
%%
data = readmatrix('robot_data10.3.csv');
%actual_current = data(500:1350, 44:49);
target_pos = data(500:1350, 2:7);
actual_pos = data(500:1350, 32:37);
target_vel = data(500:1350, 8:13);
actual_vel = data(500:1350, 38:43);

figure;
for joint = 1:6
    subplot(6,1,joint);
    plot(target_pos(:, joint), 'b-', 'LineWidth', 1.5); hold on;
    plot(actual_pos(:, joint), 'r--', 'LineWidth', 1.5); %hold off;
    xlabel('Sample Index');
    ylabel(sprintf('Joint %d Position', joint));
    title(sprintf('10.3'));
    legend('Actual_vel','actual pos');
end
disp('10.3')
[rise_time, settling_time, overshoot, steady_state_error] = compute_metrics(target_pos(:, 1), actual_pos(:, 1))
%%
data = readmatrix('robot_data_fric_comp.csv');
%actual_current = data(:, 44:49);
target_pos = data(1900:2330, 2:7);
actual_pos = data(1900:2330, 32:37);
target_vel = data(1900:2330, 8:13);
actual_vel = data(1900:2330, 38:43);

figure;
for joint = 1:6
    subplot(6,1,joint);
    plot(target_pos(:, joint), 'b-', 'LineWidth', 1.5); hold on;
    plot(actual_pos(:, joint), 'r--', 'LineWidth', 1.5); %hold off;
    xlabel('Sample Index');
    ylabel(sprintf('Joint %d Position', joint));
    title(sprintf('fric comp'));
    legend('Actual_vel','actual pos');
end
disp('fric comp')
[rise_time, settling_time, overshoot, steady_state_error] = compute_metrics(target_pos(:, 1), actual_pos(:, 1))
%%
data = readmatrix('robot_data_no_fric.csv');
actual_current = data(:, 44:49);
target_pos = data(800:1250, 2:7);
actual_pos = data(800:1250, 32:37);
target_vel = data(800:1250, 8:13);
actual_vel = data(800:1250, 38:43);

figure;
for joint = 1:6
    subplot(6,1,joint);
    plot(target_pos(:, joint), 'b-', 'LineWidth', 1.5); hold on;
    plot(actual_pos(:, joint), 'r--', 'LineWidth', 1.5); %hold off;
    xlabel('Sample Index');
    ylabel(sprintf('Joint %d Position', joint));
    title(sprintf('no fric'));
    legend('Actual_vel','actual pos');
end
disp('no fric0.0.')
[rise_time, settling_time, overshoot, steady_state_error] = compute_metrics(target_pos(:, 1), actual_pos(:, 1))
%%
function [rise_time, settling_time, overshoot, steady_state_error] = compute_metrics(target, actual)
    % Calculate the error
    %error = actual - target;
    final_value = target(1);
    error = abs(final_value - actual);
    fprintf('final value is %d ref', final_value)
    fprintf('overshoot %d ref', max(actual))

    % Rise time 10% to 90% of the final value
    rise_time_start = find(actual >= 0.1 * final_value, 1, 'first');
    rise_time_end = find(actual <= 0.9 * final_value, 1, 'first');
    rise_time = (rise_time_end - rise_time_start); % Rise time between 10% and 90% of the final value
    
    % Settling time wthin 2% of final value
    settling_time = find(abs(actual) <= 0.02 * final_value, 1, 'last');
    
    overshoot = max(actual) - final_value;  % 2% of the final value

    steady_state_error = abs(actual(end)-final_value);
end
