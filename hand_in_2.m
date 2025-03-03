clc, clear, close all

% Time vector
td = 0.002; % Seconds
t_end = 25; % Seconds
t = 0:td:t_end; % Seconds

% Desired motion
%xref = 0.3 * sin(t/3);
%yref = 0.3 * sin(t/3) .* cos(t/3);
%zref = 0.1 * sin(t);
qref = 1;
% start pose for end effector
xstart = 0.3 * sin(0/3);
ystart = 0.3 * sin(0/3) * cos(0/3);
zstart = 0.1 * sin(0);

% Transformation to compliance frame
Te_d = [1, 0, 0, 0;
        0, 1, 0, 0.1;
        0, 0, 1, 0.2;
        0, 0, 0, 1];

% Gain selection
Mp = eye(3) * 1;
Kp = eye(3) * 100;
dp = 2 * sqrt(Mp(1,1) * Kp(1,1));
Dp = eye(3) * dp;

Mo = eye(3) * 1;
Ko = eye(3) * 100;
do = 2 * sqrt(Mo(1,1) * Ko(1,1));
Do = eye(3) * do;

f_all_sensor = [];
mu_d_all_sensor = [];
f_all = [];
mu_d_all = [];
delta_ddp_cd =  [0;0;0];
delta_dp_cd = [0;0;0];
delta_p_cd =  [0;0;0];

deltaddphi_cd = [0;0;0];
deltadphi_cd = [0;0;0];
deltaphi_cd = [0;0;0];

p_c = [xstart; ystart; zstart];
p_c_all = p_c ;

p_d_all = [xstart; ystart; zstart];

phi_d = rotm2eul( eye(3), 'ZYZ')';
phi_c_all = [0;0;0];

for i = t
    p_d = desired_motion(i);
    p_d_all = [p_d_all, p_d];

    h_e = desired_wrench(i);
    f_all_sensor = [f_all_sensor h_e(4:6)];
    mu_d_all_sensor = [mu_d_all_sensor h_e(1:3)];

    h_d = wrench_trans(h_e, Te_d);
    
    f = h_d(4:6);
    mu_d = h_d(1:3);
    f_all = [f_all f];
    mu_d_all = [mu_d_all mu_d];
   
    
    delta_ddp_cd = inv(Mp) * (-Dp * delta_dp_cd - Kp * delta_p_cd + f) ;
    delta_dp_cd = delta_dp_cd + delta_ddp_cd*td; 
    delta_p_cd = delta_p_cd + delta_dp_cd*td;
    
    p_c = delta_p_cd + p_d;
    p_c_all = [p_c_all, p_c];


    deltaddphi_cd = inv(Mo)*(-Do * deltadphi_cd -Ko * deltaphi_cd+mu_d);
    deltadphi_cd = deltaphi_cd + deltaddphi_cd * td;
    deltaphi_cd = deltaphi_cd + deltadphi_cd * td;

    phi_c = deltaphi_cd+phi_d;
    phi_c_all = [phi_c_all, phi_c];
end
phi_c_all = phi_c_all(:,2:end);
p_c_all = p_c_all(:,2:end);
p_d_all = p_d_all(:,2:end);
%make plot with desired and actual motion
figure
plot3(p_c_all(1,:), p_c_all(2,:), p_c_all(3,:), Color='red')
hold on
plot3(p_d_all(1,:), p_d_all(2,:), p_d_all(3,:),  Color='blue')
grid on
xlabel("x")
ylabel("y")
zlabel("z")
legend("Desired motion", "Actual motion")
sgtitle("Motion")

%make plot with desired and actual orientation with 3 subplots

figure
subplot(3,1,1)
plot(t, phi_c_all(1,:),"Color","red")
hold on
plot(t, ones(size(t)) * phi_d(1),"Color","blue")
title("Orientation Z")
legend("Actual","Desired")
xlabel("Time [s]")
ylabel("Angle [rad]")
grid on
subplot(3,1,2)
plot(t, phi_c_all(2,:),"Color","red")
hold on
plot(t, ones(size(t)) * phi_d(2),"Color","blue")
title("Orientation Y")
xlabel("Time [s]")
ylabel("Angle [rad]")
grid on
subplot(3,1,3)
plot(t, phi_c_all(3,:),"Color","red")
hold on
plot(t, ones(size(t)) * phi_d(3), "Color","blue")
title("Orientation Z")
xlabel("Time [s]")
ylabel("Angle [rad]")
grid on
sgtitle("Orientation ZYZ")

% plot forces in 1 subplot and moments in another
figure
subplot(4,1,1)
plot(t, f_all(1,:),"Color","red")
hold on
plot(t, f_all(2,:),"Color","green")
plot(t, f_all(3,:),"Color","blue")
title("Forces at desired frame")
legend("x","y","z")
xlabel("Time [s]")
ylabel("Force [N]")
grid on
subplot(4,1,2)
plot(t, mu_d_all(1,:),"Color","red","LineWidth",1.5)
hold on
plot(t, mu_d_all(2,:),"Color","green")
plot(t, mu_d_all(3,:),"Color","blue")
title("Torque at desired frame")
legend("x","y","z")
xlabel("Time [s]")
ylabel("Torque [Nm]")
grid on
subplot(4,1,3)
plot(t, f_all_sensor(1,:),"Color","red")
hold on
plot(t, f_all_sensor(2,:),"Color","green")
plot(t, f_all_sensor(3,:),"Color","blue")
title("Forces at sensor")
legend("x","y","z")
xlabel("Time [s]")
ylabel("Force [N]")
grid on
subplot(4,1,4)
plot(t, mu_d_all_sensor(1,:),"Color","red","LineWidth",1.5)
hold on
plot(t, mu_d_all_sensor(2,:),"Color","green")
plot(t, mu_d_all_sensor(3,:),"Color","blue")
title("Torque at sensor")
legend("x","y","z")
xlabel("Time [s]")
ylabel("Torque [Nm]")
grid on
sgtitle("Forces and torque")


%plot position in 3 subplots
figure
subplot(3,1,1)
plot(t, p_c_all(1,:),"Color","red")
hold on
plot(t, p_d_all(1,:),"Color","blue")
legend("Actual","Desired")
xlabel("Time [s]")
ylabel("Position x [m]")
grid on
subplot(3,1,2)
plot(t, p_c_all(2,:),"Color","red")
hold on
plot(t, p_d_all(2,:),"Color","blue")
xlabel("Time [s]")
ylabel("Position y [m]")
grid on
subplot(3,1,3)
plot(t, p_c_all(3,:),"Color","red")
hold on
plot(t, p_d_all(3,:),"Color","blue")
xlabel("Time [s]")
ylabel("Position z [m]")
grid on
sgtitle("Position")

function h = desired_wrench(t)
    if t <= 5
        f = [0 0 0]';
        mu = [0 0 0]';
    elseif t <= 10
        f = [1 2 3]';
        mu = [0 0 0]';
    elseif t <= 15
        f = [0 0 0]';
        mu = [0 0 0]';
    elseif t <= 20
        f = [0 0 0]';
        mu = [1 0.5 1]';
    else
        f = [0 0 0]';
        mu = [0 0 0]';
    end
    h = [mu; f];
end

function wrench_B = wrench_trans(wrench_A, T_A_B)
    % Wrench: [mu; f]
    % T_A_B: Transformation of {B} given in {A}.
    R_A_B = T_A_B(1:3, 1:3);
    p_A_B = T_A_B(1:3, 4);

    adjTab = zeros(6, 6); % insert here
    adjTab(1:3, 1:3) = R_A_B;
    adjTab(4:6, 1:3) = skew_symmetric(p_A_B) * R_A_B;
    adjTab(4:6, 4:6) = R_A_B;
    
    wrench_B = adjTab' * wrench_A; % insert here
end

function mat = skew_symmetric(vector)
    mat = zeros(3, 3);
    mat(1,2) = -vector(3);
    mat(1,3) =  vector(2);
    mat(2,3) = -vector(1);
    mat(2,1) =  vector(3);
    mat(3,1) = -vector(2);
    mat(3,2) =  vector(1);
end


% Desired motion
function p_d = desired_motion(t)
 p_d = [xref(t); yref(t); zref(t)];
end

function x = xref(t)
 x = 0.3 * sin(t/3);
end

function y = yref(t)
 y = 0.3 * sin(t/3) * cos(t/3);
end

function z = zref(t)
 z = 0.1 * sin(t);
end






