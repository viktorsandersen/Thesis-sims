%% Load Robot Model and Prepare Dynamics Inputs
robotStruct = load("ur5e.mat");
robot = robotStruct.robotUR5e;

% Transpose so that each column represents a single configuration (6x1)
q = filtered_pos';
dq = filtered_vel';
ddq = filtered_acc';

% Number of samples (each sample is a single configuration)
num_samples = size(q, 2);

% Preallocate arrays to store dynamics outputs for each time sample
grav_all = zeros(6, num_samples);
tau_model  = zeros(6, num_samples);
jac_all  = cell(1, num_samples);
inertia_all = cell(6, num_samples);
vel_prod_all = zeros(6, num_samples);

% Compute dynamics for each time sample
for i = 1:num_samples
    q_i = q(:, i);
    dq_i = dq(:, i);
    ddq_i = ddq(:, i);
    
    % Compute gravitational torques, Jacobian, mass matrix, and velocity product
    grav_all(:, i) = robot.gravityTorque(q_i);
    jac_all{i} = robot.geometricJacobian(q_i, 'end_effector');
    inertia_all{i} = robot.massMatrix(q_i);
    vel_prod_all(:, i) = robot.velocityProduct(q_i, dq_i);
    
    % Compute joint torques for the current sample
    tau_model(:, i) = inertia_all{i} * ddq_i + vel_prod_all(:, i) + grav_all(:, i);
end

% Now, tau_all contains the computed joint torques for all time samples.
figure;
for joint = 1:6
    subplot(6,1,joint);
    plot(t, tau_model(joint, :), 'LineWidth', 1.5); hold on;
    plot(t, avg_phase_torque(:, joint), 'LineWidth', 1.5); hold off;
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    title(sprintf('Joint %d Torque', joint));
end


%% Regressor matrix
tau_fric = avg_phase_torque' - tau_model; %Measurement - model?
[numJoints, numSamples] = size(dq);


Y = zeros(numJoints * numSamples, 2 * numJoints);

for k = 1:numSamples
    for i = 1:numJoints
        % Compute the overall row index corresponding to joint i at sample k.
        rowIdx = (k - 1) * numJoints + i;
        % Set the entry corresponding to the Coulomb friction coefficient for joint i.
        Y(rowIdx, i) = sign(dq(i, k));
        % Set the entry corresponding to the viscous friction coefficient for joint i.
        Y(rowIdx, numJoints + i) = dq(i, k);
    end
end

% Now Y is the regressor matrix with size (6*5000) x 12.

%%
% Reshape tau_friction into a column vector (each row of Y corresponds to one measurement)
tau_friction_vec = reshape(tau_fric, [], 1);  % (6*5000) x 1 vector

% Solve for the parameter vector using least squares
pi = Y \ tau_friction_vec;

% Extract the friction coefficients for each joint
f_coulomb = pi(1:6);      % Estimated Coulomb friction coefficients (for joints 1 to 6)
f_viscous  = pi(7:12);     % Estimated Viscous friction coefficients (for joints 1 to 6)

% Display the estimated parameters
disp('Estimated Coulomb friction coefficients:');
disp(f_coulomb);
f_c_new = f_coulomb + 1.0;
f_v_new = f_viscous + 1.0;
pi_new = pi + 1.0;
disp('Estimated Viscous friction coefficients:');
disp(f_viscous);
%%
% Ting der kunne være forkert:
% FFT fjernelse af støj
% Tau_friction_est udregning
% Måske WLS i stedet for LS (\)
% Hvis intet, kig på autocovariance
size(Y)
size(pi)
tau_friction_est = reshape(Y * pi, numJoints, numSamples);
% pi = 12x1, Y=30k x 12
%tau_friction_sum = sum(Y * pi);  % This will give you a 12 x 1 vector
tau_full = tau_model + tau_friction_est;
for joint = 1:6
    subplot(6,1,joint);
    plot(t, tau_full(joint, :), 'LineWidth', 1.5); hold on;
    plot(t, avg_phase_torque(:, joint), 'LineWidth', 1.5); hold off;
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    title(sprintf('Joint %d Torque', joint));
    legend('Tau model + friction', 'Averaged measured torques');
end