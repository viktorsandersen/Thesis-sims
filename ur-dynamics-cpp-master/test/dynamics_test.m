clc
clear
close all

%%
robot = load("ur3e.mat");
robot = robot.robotUR3e;

q = [1.0000,   1.0472,    1.0472, 1.0472, 1.0472, 1.0472].';
dq = [1.0000,   1.0472,    1.0472, 1.0472, 1.0472, 1.0472].';
ddq = [1.0000,   1.0472,    1.0472, 1.0472, 1.0472, 1.0472].';

%%
grav = robot.gravityTorque(q)
jac = robot.geometricJacobian(q, 'end_effector')
inertia = robot.massMatrix(q)
vel_prod = robot.velocityProduct(q, dq)