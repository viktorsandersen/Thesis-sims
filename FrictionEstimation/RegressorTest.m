%% Load Robot Model and Prepare Dynamics Inputs
robotStruct = load("ur5e.mat");
robot = robotStruct.robotUR5e;

% Define joint positions (q), velocities (qdot), and accelerations (qddot)
q = [0.5; -0.3; 0.8; -0.2; 0.1; -0.6];       % Example joint positions (6x1 vector)
qdot = [0.1; -0.05; 0.15; -0.1; 0.2; -0.3];  % Example joint velocities (6x1 vector)
qddot = [0.05; -0.02; 0.1; -0.05; 0.07; -0.1]; % Example joint accelerations (6x1 vector)

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

% Display the actual torques
disp('Actual joint torques from robot model (tau_model):');
disp(tau_model);

% Define a simple regressor matrix Y (6x6) for testing (this should match the model form)
% For simplicity, we will construct a dummy regressor matrix based on q, qdot, and qddot
Y = [q, qdot, qddot];  % A simple regressor matrix [q, qdot, qddot] (6x3)

% Define a dummy parameter vector pi (3x1)
% For this example, we are just assuming arbitrary values for the parameters
pi = [0.5; 0.2; 0.1];  % Dummy values for testing

% Predicted torques using the regressor matrix and parameters
tau_pred = Y * pi;  % Predicted torques (6x1)

% Display the predicted torques
disp('Predicted joint torques using regressor matrix (tau_pred):');
disp(tau_pred);

% Compute the error between the actual model and the predicted torque
error = norm(tau_model - tau_pred);

% Display the error
disp(['Error between actual and predicted torques: ', num2str(error)]);
