data = readmatrix('data/robot_data_ur5_full_speed_wrench_first_5k.csv');
external_joint_torques = data(:, end-5:end);
figure;
for joint = 1:6
    subplot(6,1,joint);
    plot(external_joint_torques(:, joint));
    xlabel('Sample Index');
    ylabel(sprintf('Torque (Nm) - Joint %d', joint));
    title(sprintf('External Joint Torque - Joint %d', joint));
end

actual_current = data(:, 44:49);
figure;
for i = 1:6
    subplot(6, 1, i);
    plot(actual_current(:, i));
    xlabel('Sample Index');
    ylabel(sprintf('Current %d (A)', i-1));
    title(sprintf('Actual Current %d', i-1));
end

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
