clc
clear 
close all

%% 
Fs = 500;

reg_mat = @regressor_matrix_ur10e;

% Yarg = reg_mat(ones(1,6), ones(1,6), ones(1,6))
% path = "./";
% file_1 = "sys_id_ur10_1.csv";
path = "./data/";
file_1 = "recording_sysid_310322_1.csv";
file_2 = "recording_sysid_310322_2.csv";

opts_actual = detectImportOptions(path + file_1);
opts_target = opts_actual;
opts_target.SelectedVariableNames = {'timestamp', ...
    'target_q_0', 'target_q_1', 'target_q_2', 'target_q_3', 'target_q_4', 'target_q_5', ...
    'target_qd_0', 'target_qd_1', 'target_qd_2', 'target_qd_3', 'target_qd_4', 'target_qd_5', ...
    'target_current_0', 'target_current_1', 'target_current_2', 'target_current_3', 'target_current_4', 'target_current_5' ...
    'target_moment_0', 'target_moment_1', 'target_moment_2', 'target_moment_3', 'target_moment_4', 'target_moment_5' ...
    'target_qdd_0', 'target_qdd_1', 'target_qdd_2', 'target_qdd_3', 'target_qdd_4', 'target_qdd_5', ...
    };
opts_actual.SelectedVariableNames = {'timestamp', ...
    'actual_q_0', 'actual_q_1', 'actual_q_2', 'actual_q_3', 'actual_q_4', 'actual_q_5', ...
    'actual_qd_0', 'actual_qd_1', 'actual_qd_2', 'actual_qd_3', 'actual_qd_4', 'actual_qd_5', ...
    'actual_current_0', 'actual_current_1', 'actual_current_2', 'actual_current_3', 'actual_current_4', 'actual_current_5' ...
    'joint_temperatures_0', 'joint_temperatures_1', 'joint_temperatures_2', 'joint_temperatures_3', 'joint_temperatures_4', 'joint_temperatures_5' ...
    'actual_joint_voltage_0', 'actual_joint_voltage_1', 'actual_joint_voltage_2', 'actual_joint_voltage_3', 'actual_joint_voltage_4', 'actual_joint_voltage_5' ...
    };

% training file
data_1_actual = readtable(path + file_1, opts_actual);
data_1_target = readtable(path + file_1, opts_target);
data_1_times = data_1_actual{:, 1} - data_1_actual{1, 1};

% test file
data_2_actual = readtable(path + file_2, opts_actual);
data_2_target = readtable(path + file_2, opts_target);
data_2_times = data_2_actual{:, 1} - data_2_actual{1, 1};

%%
% training file
q = [data_1_actual.actual_q_0, ...
     data_1_actual.actual_q_1, ...
     data_1_actual.actual_q_2, ...
     data_1_actual.actual_q_3, ...
     data_1_actual.actual_q_4, ...
     data_1_actual.actual_q_5];

dq = [data_1_actual.actual_qd_0, ...
      data_1_actual.actual_qd_1, ...
      data_1_actual.actual_qd_2, ...
      data_1_actual.actual_qd_3, ...
      data_1_actual.actual_qd_4, ...
      data_1_actual.actual_qd_5];

ddq = [data_1_target.target_qdd_0, ...
       data_1_target.target_qdd_1, ...
       data_1_target.target_qdd_2, ...
       data_1_target.target_qdd_3, ...
       data_1_target.target_qdd_4, ...
       data_1_target.target_qdd_5];

currents = [data_1_actual.actual_current_0, ...
    data_1_actual.actual_current_1, ...
    data_1_actual.actual_current_2, ...
    data_1_actual.actual_current_3, ...
    data_1_actual.actual_current_4, ...
    data_1_actual.actual_current_5];

% test file
q_test = [data_2_actual.actual_q_0, ...
     data_2_actual.actual_q_1, ...
     data_2_actual.actual_q_2, ...
     data_2_actual.actual_q_3, ...
     data_2_actual.actual_q_4, ...
     data_2_actual.actual_q_5];

dq_test = [data_2_actual.actual_qd_0, ...
      data_2_actual.actual_qd_1, ...
      data_2_actual.actual_qd_2, ...
      data_2_actual.actual_qd_3, ...
      data_2_actual.actual_qd_4, ...
      data_2_actual.actual_qd_5];

ddq_test = [data_2_target.target_qdd_0, ...
       data_2_target.target_qdd_1, ...
       data_2_target.target_qdd_2, ...
       data_2_target.target_qdd_3, ...
       data_2_target.target_qdd_4, ...
       data_2_target.target_qdd_5];

currents_test = [data_2_actual.actual_current_0, ...
    data_2_actual.actual_current_1, ...
    data_2_actual.actual_current_2, ...
    data_2_actual.actual_current_3, ...
    data_2_actual.actual_current_4, ...
    data_2_actual.actual_current_5];

%% trim data
max_time_index = 20 * Fs;  % Use only 20s of the data
[data_1_times, q, dq, ddq, currents] = trim_data(data_1_times, q, dq, ddq, currents, max_time_index);
[data_2_times, q_test, dq_test, ddq_test, currents_test] = trim_data(data_2_times, q_test, dq_test, ddq_test, currents_test, max_time_index);

%% Plots
figure;
ax1 = subplot(4, 1, 1);
plot(data_1_times, q)
ylabel("$q$", 'Interpreter','latex')
title("Training Data")

ax2 = subplot(4, 1, 2);
plot(data_1_times, dq)
ylabel("$\dot{q}$", 'Interpreter','latex')

ax3 = subplot(4, 1, 3);
plot(data_1_times, ddq)
ylabel("$\ddot{q}$", 'Interpreter','latex')

ax4 = subplot(4, 1, 4);
plot(data_1_times, currents)
ylabel("$\tau$", 'Interpreter','latex')
xlabel("Time [s]")

linkaxes([ax1,ax2,ax3,ax4],'x')

%
figure;
ax1 = subplot(4, 1, 1);
plot(data_2_times, q_test)
ylabel("$q$", 'Interpreter','latex')
title("Test Data")

ax2 = subplot(4, 1, 2);
plot(data_2_times, dq_test)
ylabel("$\dot{q}$", 'Interpreter','latex')

ax3 = subplot(4, 1, 3);
plot(data_2_times, ddq_test)
ylabel("$\ddot{q}$", 'Interpreter','latex')

ax4 = subplot(4, 1, 4);
plot(data_2_times, currents_test)
ylabel("$\tau$", 'Interpreter','latex')
xlabel("Time [s]")

linkaxes([ax1,ax2,ax3,ax4],'x')

%% Get regression matrix.
use_friction = false;
Ymat = get_big_regressor(reg_mat, q, dq, ddq, use_friction);

%% least squares solution:
% Y(q, dq, ddq) * pi = currents

%% Estimated currents:
% validate on test data
Ymat_test = get_big_regressor(reg_mat, q_test, dq_test, ddq_test, use_friction);
currents_est = % insert current estimation

%% Plot the estimated and the actual current.

%% Plot the estimation error.

%% Compute the autocovariance and plot it.

%% Plot the estimated currents w.r.t. dq.

%% Compute fitness.

%% Compute the condition number.
