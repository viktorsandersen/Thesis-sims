# Imports libraries 
import numpy as np
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import os
import mediapy as media
from tqdm import tqdm
import mujoco_viewer
import time as clock

# Robotics toolbox and dependencies
import roboticstoolbox as rtb
import spatialmath as sm
from spatialmath import UnitQuaternion, SE3
from spatialmath.base import q2r, r2x, rotx, roty, rotz, r2q, q2r, v2q
from mujoco_parser import MuJoCoParserClass

# Import local libraries
from Ur10e import Ur10e
from actuator import ActuatorMotor, ActuatorVelocity, ActuatorPosition, update_actuator
from quaternion_helper import *

#########
# UR_RTDE
##########
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

# Parameters
vel = 0.5
acc = 0.5
rtde_frequency = 500.0
dt = 1.0/rtde_frequency  # 2ms
flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
ur_cap_port = 50002
robot_ip = "192.168.1.129"

# ur_rtde realtime priorities
rt_receive_priority = 90
rt_control_priority = 85

rtde_r = RTDEReceive(robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)

lookahead_time = 0.1
gain = 600


r_e = rtde_r.getActualTCPPose()
v_e = rtde_r.getActualTCPSpeed()
q_e = rtde_r.getActualQ()
dq_e = rtde_r.getActualQd()
h_e = rtde_r.getActualTCPForce()
print("Force: ", h_e)
print("Position: ", r_e)
print("Velocity: ", v_e)
print("q: ", q_e)
print("dq: ", dq_e)
rtde_c.zeroFtSensor()
h_e = rtde_r.getActualTCPForce()
print("Force: ", h_e)
# Make ur10e instance
ur10e = Ur10e()

poi = np.array([
    [93, 243],
    [130, 243],
    [130, 223],
    [175, 223],
    [175, 263],
    [130, 263],
    [130, 223],
    [175, 223],
    [175, 243],
    [213, 243],
    [213, 223],
    [254, 223],
    [254, 164],
    [334, 164],
    [334, 144],
    [379, 144],
    [379, 184],
    [334, 184],
    [334, 144],
    [379, 144],
    [379, 164],
    [663, 164],
    [663, 271],
    [648, 271],
    [648, 289],
    [663, 289],
    [663, 392],
    [379, 392],
    [379, 372],
    [334, 372],
    [334, 412],
    [379, 412],
    [379, 372],
    [334, 372],
    [334, 392],
    [254, 392],
    [254, 339],
    [213, 339],
    [213, 319],
    [175, 319],
    [175, 339],
    [130, 339],
    [130, 319],
    [93, 319],
    [130, 319],
    [130, 299],
    [175, 299],
    [175, 319],
    [213, 319],
    [213, 299],
    [297, 299],
    [297, 339],
    [213, 339],
    [213, 299],
    [297, 299],
    [297, 319],
    [334, 319],
    [334, 299],
    [379, 299],
    [379, 339],
    [334, 339],
    [334, 299],
    [379, 299],
    [379, 319],
    [435, 319],
    [435, 292],
    [444, 281],
    [480, 281],
    [480, 261],
    [525, 261],
    [525, 281],
    [565, 281],
    [565, 301],
    [648, 301],
    [648, 261],
    [565, 261],
    [565, 281],
    [525, 281],
    [525, 301],
    [480, 301],
    [480, 281],
    [444, 281],
    [435, 272],
    [425, 281],
    [435, 292],
    [444, 281],
    [435, 272],
    [435, 243],
    [379, 243],
    [379, 263],
    [334, 263],
    [334, 243],
    [297, 243],
    [297, 263],
    [213, 263],
    [213, 223],
    [297, 223],
    [297, 243],
    [334, 243],
    [334, 223],
    [379, 223],
    [379, 243]
])
poi = poi + np.array([0, -281])

########################
# Generate trajectory
########################

T0 = sm.SE3.Trans(0.0, 0, 0.301) * sm.SE3.RPY([0, np.pi, 0]) # sm.SE3.Trans(0.5, 0, 0.3) * sm.SE3.RPY([0, np.pi, 0]) changed z from 0.3 to 0.4 to avoid collision
T1 = sm.SE3.Trans(0.0, 0, 0.301) * sm.SE3.RPY([0, np.pi, 0])

dt = 0.002 
endTime = 0.3
n = int(endTime/dt)

ctr = np.array([])
times = np.array([])
startTime = 0
scalingX = 5000
scalingY = 2500
movementX = 0.4
movementY = 0.0



for i in tqdm(range(0, poi.shape[0]-1)):
    distance = np.sqrt((poi[i+1, 0] - poi[i, 0])**2 + (poi[i+1, 1] - poi[i, 1])**2)
    endTime = distance*0.05
    n = int(endTime/dt)
    T0.t[0] = movementX + poi[i, 0]/scalingX
    T0.t[1] = movementY + poi[i, 1]/scalingY
    T1.t[0] = movementX + poi[i+1, 0]/scalingX
    T1.t[1] = movementY + poi[i+1, 1]/scalingY
    trajectory = rtb.ctraj(T0, T1, n)
    time = np.linspace(startTime, startTime+endTime, n)
    startTime = startTime+endTime
    if i == 0:
        ctr = trajectory
        times = time
    else:
        x = ctr.Alloc(len(ctr)+len(trajectory))
        for j in range(0, len(ctr)):
            x[j] = ctr[j]
        for j in range(0, len(trajectory)):
            x[j+len(ctr)] = trajectory[j]
        ctr = x
        times = np.append(times, time)
# Desired orientation of trajectory
quat_d = r2q(T0.R)

# get translational part of trajectory
p = np.zeros((len(ctr), 2))
for i in range(len(ctr)):
    p[i] = ctr[i].t[0:2]
# Add orientational part (quaternion) to P
p = np.hstack((p, np.zeros((len(ctr), 1)), np.ones((len(ctr), 1)), np.zeros((len(ctr), 1))))

# GRADIENTS of trajectory
# calculate gradient of p
dp = np.gradient(p, axis=0)
# calculate double gradient of p
ddp = np.gradient(dp, axis=0)

############
# Move robot to start pose
##############

# start pose
start_q = np.array([-0.30645877519716436, -1.060514287357666, 2.044889275227682, -2.556969781915182, -1.571435276662008, 3.485653877258301]) #above the table


###############
# Controller
#################

# Define the mapping of subspace of motion controller
S_v = np.diag([1.0, 1.0, 1.0, 1.0, 1.0]) #added 0 so it does not lift the 
zero_row = np.zeros((1,5))
S_v = np.insert(S_v, 2, zero_row,axis=0)
S_v_transposed = np.linalg.pinv(S_v)

# Define the gains for the motion controller
kp_trans_val = 1000
kp_rot_val = 2000
kp_trans = np.array([kp_trans_val, kp_trans_val])
kp_rot = np.array([kp_rot_val, kp_rot_val, kp_rot_val])
kd_trans = 2*np.sqrt(kp_trans)
kd_rot = 2*np.sqrt(kp_rot)
kp = np.diag(np.concatenate((kp_trans, kp_rot), axis=0))
kd = np.diag(np.concatenate((kd_trans, kd_rot), axis=0))

ki = np.eye(5)

for i in range(2):
    ki[i][i] = 2

error_ki = 0

# Define the  mapping of subspace of force controller
S_f = np.array([0,0,1,0,0,0])
#S_f_transposed = np.linalg.pinv(S_f)
S_f_transposed = S_f.transpose()

# Define the gains for the force controller
k_p_lambda_val = 0.01
m_lambda_val = 2000
k_d_lambda_val = 2*np.sqrt(m_lambda_val*k_p_lambda_val) 
k_p_lambda = np.array([k_p_lambda_val])
k_d_lambda = np.array([k_d_lambda_val])

#############################################
# Initialize the variables that get reassigned in the loop of the controller
#############################################
# Robot states
r_e = np.zeros(6)
v_e = np.zeros(6)
q_e = start_q#np.zeros(6)
dq_e = np.zeros(6)
# Jaobian
J = ur10e.jacob0(q_e)
dJ = ur10e.djacob0(q_e, dq_e, representation=None)
# Subspace 
r = np.zeros((5,1))
v = np.zeros((5,1))
# quaternion
eta_diff = np.zeros(1)
eps_diff = np.zeros(3)
quat_diff = np.zeros(4)
# motion controller output
alpha_v = np.zeros(5)
alpha_m = np.zeros(6)
alpha_m_joint = np.zeros(6)
# Force controller var
h_d = np.array([0,0,-8,0,0,0])
lambda_d = S_f_transposed @ h_d
h_e = np.array([0,0,0,0,0,0])

# Integrated dq
qvel = np.zeros(6)

# Calculate time of controll loop
start_time = clock.time()
duration = times[-10]

# Logging
test_time = 3
log = np.zeros((len(range(int(test_time*1000/dt))), 6))


##########################
# Control loop
##########################
i=0 # Index for the trajectory
while clock.time()-start_time < test_time: #duration:
    # Get states from the robot
    r_e = rtde_r.getActualTCPPose()
    v_e = rtde_r.getActualTCPSpeed()
    q_e = rtde_r.getActualQ()
    dq_e = rtde_r.getActualQd()
    h_e = rtde_r.getActualTCPForce()

    # Time log
    time_start_loop = clock.time()
    # Update orientation to use quaternion
    quat_e = v2q(r_e[3:6])
    r_e = np.concatenate((r_e[0:3], quat_e[1:]))
    # Calculate the jacobian and check safety
    J = ur10e.jacob0(q_e)
    """dJ = ur10e.djacob0(q_e, dq_e, representation=None)
    if(np.abs(np.linalg.det(J)) <= 0.001):
        print("Singular configuration ", ur10e.d.time, np.linalg.det(J))
        rtde_c.servoStop(10.0)
        break """
    # Current endeffector velocity and position in motion control subspace
    r = S_v_transposed @ r_e 
    v = S_v_transposed @ v_e 
    ############################
    # Motion controller
    ############################
    alpha_v[0:2] = ddp[i,0:2] + kd_trans @ (dp[i,0:2] - v[0:2]) + kp_trans @ (p[i,0:2] - r[0:2])
    #Compute K' for rotation i quaternion space
    eta_diff = np.array([quat_d[0]*quat_e[0] - np.dot(quat_d[1:], quat_e[1:])])
    eps_diff = quat_d[0]*quat_e[1:] - quat_e[0]*quat_d[1:] - skewSymmetric(quat_e[1:])@quat_d[1:]
    quat_diff = np.concatenate((eta_diff, eps_diff))

    K_rot = K_(quat_diff, np.diag(kp_rot))

    alpha_v[2:5] = ddp[i,2:] + np.diag(kd_rot) @ (dp[i,2:] - v[2:]) + K_rot @ (eps_diff)
    #Acceleration in cartesion space
    alpha_m = S_v @ alpha_v
    #Acceleration in joint space
    """
    alpha_m_joint = np.linalg.pinv(J) @ (alpha_m - dJ @ dq_e)
    """
    lambda_ = S_f_transposed @ -h_e
    vz = S_f_transposed @ v_e 
    f_lambda = k_p_lambda*(lambda_d-lambda_) - k_d_lambda * 5 * vz
    
    alpha_f = S_f * f_lambda
    """
    #Force in joint space
    alpha_f_joint = np.linalg.pinv(J) @ (alpha_f - dJ @ dq_e)
    #Total acceleration joint space
    alpha = alpha_m_joint + alpha_f_joint
    qvel = qvel + alpha * dt
    """
    """
    rtde_c.speedJ(qvel, acceleration = 0.5, time = 0.0)
    """

    time_end_loop = clock.time()
    log[i] = time_end_loop - time_start_loop

    i = i + 1

    

# plot the log
# cut off log to i
print(i)
log = log[0:i]
plt.plot(log)
plt.show()

# print mean and std of log
print("Mean: ", np.mean(log))
print("Std: ", np.std(log))
print("Max: ", np.max(log))








