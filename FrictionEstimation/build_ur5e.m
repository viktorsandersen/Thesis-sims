clc
clear
close all

%% UR5e Parameters

% DH parameters for UR5e (in meters and radians)
a = [0, -0.425, -0.392, 0, 0, 0];
d = [0.1625, 0, 0, 0.1333, 0.0997, 0.996];
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];
% (No constant joint offsets for this DH table)
theta = [0, 0, 0, 0, 0, 0];

% Mass (kg) for each link
m = [3.761, 8.058, 2.846, 1.37, 1.3, 0.365];

% Center of Mass for each link (expressed in the link coordinate frame, in meters)
pl1 = [0, -0.02561, 0.00193];
pl2 = [0.2125, 0, 0.11336];
pl3 = [0.15, 0, 0.0265];
pl4 = [0, -0.0018, 0.01634];
pl5 = [0, 0.0018, 0.01634];
pl6 = [0, 0, -0.001159];
CoM = {pl1', pl2', pl3', pl4', pl5', pl6'};

% Inertia matrices about the COM for each link (in kg*m^2)
I1 = diag([0.0, 0.0, 0.0]);
I2 = diag([0.0, 0.0, 0.0]);
I3 = diag([0.0, 0.0, 0.0]);
I4 = diag([0.0, 0.0, 0.0]);
I5 = diag([0.0, 0.0, 0.0]);
I6 = diag([0.0, 0.0, 0.0002]);
Inertia = {I1, I2, I3, I4, I5, I6};

% Build DH parameter table (the fourth column is the joint offset, here zero)
dhparams = [a' alpha' d' zeros(6, 1)];

%% Create UR5e Robot Model

robotUR5e = rigidBodyTree;
robotUR5e.DataFormat = 'column';
robotUR5e.Gravity = [0, 0, -9.82]';

% Preallocate cell arrays for bodies and joints
bodies = cell(6,1);
joints = cell(6,1);

N = 6;
for i = 1:N
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)], "revolute");
    
    % Set the fixed transform using the DH parameters
    setFixedTransform(joints{i}, dhparams(i,:), "dh");
    
    bodies{i}.Joint = joints{i};
    bodies{i}.Mass = m(i);
    bodies{i}.CenterOfMass = CoM{i}';  % Transpose to a row vector
    
    % Adjust the inertia to the link frame using the parallel axis theorem
    Inert = Inertia{i} + (CoM{i}' * CoM{i} * eye(3) - (CoM{i} * CoM{i}')) * m(i);
    bodies{i}.Inertia = [Inert(1,1), Inert(2,2), Inert(3,3), Inert(2,3), Inert(1,3), Inert(1,2)];
    
    if i == 1
        addBody(robotUR5e, bodies{i}, "base")
    else
        addBody(robotUR5e, bodies{i}, bodies{i-1}.Name)
    end
end

% Add an end-effector body
bodyEndEffector = rigidBody('end_effector');
bodyEndEffector.Mass = 0;
bodyEndEffector.Inertia = zeros(1,6);
% Define an arbitrary transformation for the end effector (adjust as needed)
tform_ee = trvec2tform([0, 0, 0]);  
setFixedTransform(bodyEndEffector.Joint, tform_ee);
addBody(robotUR5e, bodyEndEffector, robotUR5e.Bodies{end}.Name);

% Display robot details
showdetails(robotUR5e)
interactiveRigidBodyTree(robotUR5e, MarkerScaleFactor=0.5);

%% Save the UR5e Model
fname = sprintf("ur5e.mat");
save(fname, "robotUR5e");
