#!/usr/bin/env python3
import sys
import numpy as np
sys.path.append("..")
import logging
from cvxopt import matrix, solvers
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import threading
import sys
import termios
import tty
import pandas as pd
np.set_printoptions(precision=8, suppress=True)
import ur_robot
robot = ur_robot.URRobot(ur_robot.RobotType.UR5e)
import subprocess
import signal
import os

# Start record.py as a background process
log_folder = "SafetyAware"
log_name = "NoPushes"

# Ensure the folder exists
os.makedirs(log_folder, exist_ok=True)

log_filename_rtde = os.path.join(log_folder, f"{log_name}RTDE.csv")
log_filename_local = os.path.join(log_folder, f"{log_name}.csv")

record_process = subprocess.Popen([
    'python3', 'record.py',
    '--frequency', '500',
    '--output', log_filename_rtde
])
#q = np.array([[1.0000,   1.0472,    1.0472, 1.0472, 1.0472, 1.0472]]).T
def check_for_q():
    global keep_running
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while keep_running:
            if sys.stdin.read(1) == 'q':
                keep_running = False
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def skew_symmetric(mat):
    return 0.5 * (mat - mat.T)
 
def adjoint_transform(H):
    R = H[:3, :3]
    p = H[:3, 3]
    p_hat = np.array([[0, -p[1], p[0]],
                      [p[2], 0, -p[0]],
                      [-p[2], p[1], 0]])
    upper = np.hstack((R, np.zeros((3, 3))))
    lower = np.hstack((p_hat @ R, R))
    return np.vstack((upper, lower))
 
def vee(S):
    return np.array([
        S[1, 0],
        S[0, 2],
        S[2, 1]
    ])


def setp_to_list(sp):
    sp_list = []
    for i in range(0, 6):
        sp_list.append(sp.__dict__["input_double_register_%i" % i])
    return sp_list


def list_to_setp(sp, list):
    for i in range(0, 6):
        sp.__dict__["input_double_register_%i" % i] = list[i]
    return sp


def unpack_mass_matrix_and_coriolis(state):
    mass_flat = [getattr(state, f'output_double_register_{i}') for i in range(0, 36)]
    mass_mat = [mass_flat[i:i+6] for i in range(0, 36, 6)]
    coriolis = [getattr(state, f'output_double_register_{i}') for i in range(36, 42)]
    return mass_mat, coriolis

MAX_TORQUE = np.array([10, 10, 10, 5, 5, 5])
MIN_TORQUE = np.array([-10, -10, -10, -5, -5, -5])
torques = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# logging
time_log = []
nominal_torque_log = np.zeros((0, 6))
safe_torque_log = np.zeros((0, 6))

Vpot_log = np.zeros(0)
Tkin_log = np.zeros(0)
Etot_log = np.zeros(0)
lambda_log = np.zeros(0)
Pc_log = np.zeros(0)
Pcg_log = np.zeros(0)
PcTot_log = np.zeros(0)
beta_log = np.zeros(0)
K_log = np.zeros((0, 6))
Hvt_log = np.zeros((0, 6))
#UR_stiffness_log = np.zeros((0, 6))
#UR_damping_log = np.zeros((0, 6))
H_n_log = np.zeros((0, 6))
u_n_log = np.zeros((0, 6))
tau_on_log = np.zeros((0, 6))
s_n_log = np.zeros((0, 6))
s_dot_log = np.zeros((0, 6))
qdot_log = np.zeros((0, 6))
H_n_cart_log = np.zeros((0, 6))
u_n_cart_log = np.zeros((0, 6))
tau_on_cart_log = np.zeros((0, 6))
s_n_cart_log = np.zeros((0, 6))
s_dot_cart_log = np.zeros((0, 6))
v_act_cart_log = np.zeros((0, 6))
v_act_pos_log = np.zeros((0, 6))
u_log = np.zeros((0, 6))
H_n_total_log = np.zeros(0)
H_n_cart_total_log = np.zeros(0)
Pc_cart_log = np.zeros(0)
beta_cart_log = np.zeros(0)
Vt_log = np.zeros(0)
Vo_log = np.zeros(0)
dEtot_log = np.zeros(0)
W0_log = np.zeros((0, 6))
v_act_log = np.zeros((0, 6))
Pcn_log = np.zeros((0, 6))
Pcn_cart_log = np.zeros((0, 6))
c_fric_log = np.zeros((0, 6))
v_fric_log = np.zeros((0, 6))
c_fric_cart_log = np.zeros((0, 6))
v_fric_cart_log = np.zeros((0, 6))
c_fric_sum_log = np.zeros(0)
v_fric_sum_log = np.zeros(0)
c_fric_cart_sum_log = np.zeros(0)
v_fric_cart_sum_log = np.zeros(0)
Pcn_sum_log = np.zeros(0)
Pcn_cart_sum_log = np.zeros(0)
tau_nom_log = np.zeros((0, 6))
power_v_fric_log = np.zeros((0, 6))
power_c_fric_log = np.zeros((0, 6))
power_fric_log = np.zeros((0, 6))
full_power_fric_log = np.zeros((0, 6))
tc_log = np.zeros(0)
wrenchAtTool_log = np.zeros((0, 6))
ft_error_log = np.zeros((0, 6))
ftBaseFrame_log = np.zeros((0, 6))

def pose_to_transformation(pose):
    """
    Convert a [6,1] pose vector (axis-angle) to a 4x4 transformation matrix.
    
    pose: [x, y, z, Rx, Ry, Rz] where (Rx, Ry, Rz) is the axis-angle vector.
    """
    # Flatten pose to a 1D array just in case it's column-shaped
    pose = pose.flatten()
    
    # Extract position and rotation vector
    position = pose[:3]
    rotvec = pose[3:]

    # Create rotation matrix from axis-angle vector
    rotation = R.from_rotvec(rotvec).as_matrix()

    # Build the 4x4 transformation matrix
    transformation = np.eye(4)
    transformation[:3, :3] = rotation
    transformation[:3, 3] = position

    return transformation

# Energy tank log
#Etot_log = np.zeros(0)
#lambda_s_log = np.zeros(0)
#Pc_log = np.zeros(0)
#beta_s_log = np.zeros(0)
trans_error_log = np.zeros(0)
orientation_error_log = np.zeros(0)
# logging.basicConfig(level=logging.INFO)

#ROBOT_HOST = "172.17.0.2" #URsim
ROBOT_HOST = "192.168.0.100"
ROBOT_PORT = 30004
config_filename = "control_loop_configuration.xml"

keep_running = True

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

for i in range(0,10):
        setattr(setp, f'input_double_register_{i}', 0.0)
con.send(setp)
if not con.send_start():
    sys.exit()

# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
watchdog.input_int_register_0 = 0

#### Energy tank #####
# K (stiffness), B (damping) - kendt
# Hvt (error from end-effector to desired position) 1x6
# q, qdot = 2x6 = 12
# Til pc: wrench 1x6
# Til beta: Pmax 1x1
# 

# enter critical
# RTDE kan se tiden på control loop

Kt = np.eye(3) * 100
Ko = np.eye(3) * 10
Bo = np.eye(3) * 20
Bt = np.eye(3) * 6.32
Kc = np.zeros((3, 3))
K_full_to = np.block([[Kt, Kc], [Kc.T, Ko]])
B_full_to = np.block([[Bt, Kc], [Kc.T, Bo]])

Go = 0.5 * (np.trace(Ko)) * np.eye(3) - Ko
#print("Go", Go)
Go_diag = [Go[0, 0], Go[1, 1], Go[2, 2]]
Gt = 0.5 * (np.trace(Kt)) * np.eye(3) - Kt
#Gc = 0.5 * (np.trace(Kc)) * np.eye(3) - Kc

t = 0.0
idx = 0
Emax = 3.0
Pmax = 2.0
Kt_new = Kt
Ko_new = Ko
Etot = 0
Pc = 0
s_n_val = 5
s_n_start = np.ones(6) * s_n_val
dt = 0.002
s_n = np.ones(6) * s_n_val
s_n_cart = np.ones(6) * s_n_val
u_n_cart = np.zeros(6)
u_n = np.zeros(6)
previous_q = np.zeros(6)
move_completed = True
Etot_prev = None
dEtot = 0
#coulomb = np.array([4.76, 4.73, 3.36, 1.36, 1.5, 1.51])
#viscous = np.array([9.79, 6.0, 4.5, 1.5, 1.4, 3.2])
coulomb = np.array([9.1, 4.7, 6.8, 1.5, 1.8, 1.7])
viscous = np.array([13.3, 5.6, 6.8, 1.5, 1.9, 2.5])
accumulated_dissipated_energy_per_joint = 0
H_n_start = 0.5 * (s_n_start**2)
H_n_total_start = np.sum(H_n_start)
ft_sum = 0
threading.Thread(target=check_for_q, daemon=True).start()
print("Press 'q' to quit")
#opnedopned, 76,2, 74
while keep_running:
    state = con.receive()
    if state is None:
        break
    #mass_mat, vel = unpack_mass_matrix_and_coriolis(state)
    #print(f"Joint vel: {vel}")

    Hvt = [getattr(state, f'output_double_register_{i}') for i in range(42,48)]
    q = [getattr(state, f'output_double_register_{i}') for i in range(36,42)]
    q = np.array(q).reshape(6, 1)
    qdot = [getattr(state, f'output_double_register_{i}') for i in range(30,36)]
    #ft_error = [getattr(state, f'output_double_register_{i}') for i in range(30,36)]
    v_act = [getattr(state, f'output_double_register_{i}') for i in range(24,30)]
    tau = [getattr(state, f'output_double_register_{i}') for i in range(18,24)]
    #timecounter = getattr(state, f'output_double_register_10')
    #tc_log = np.append(tc_log, [timecounter], axis=0)
    ft_error = [getattr(state, f'output_double_register_{i}') for i in range(0,6)]
    wrenchAtTool = [getattr(state, f'output_double_register_{i}') for i in range(12,18)]
    #UR_stiffness = [getattr(state, f'output_double_register_{i}') for i in range(12,18)]
    #UR_damping = [getattr(state, f'output_double_register_{i}') for i in range(0,6)]
    current_pose = [getattr(state, f'output_double_register_{i}') for i in range(6,12)]
    current_pose = np.array(current_pose).reshape(6, 1)
     
    ft_error_log = np.append(ft_error_log, [ft_error], axis=0)
    wrenchAtTool_log = np.append(wrenchAtTool_log, [wrenchAtTool], axis=0)
    #Pmax =getattr(state, f'output_double_register_0')
    #Emax =getattr(state, f'output_double_register_1')
    #lambda_s =getattr(state, f'output_double_register_2')
    #Etot = getattr(state, f'output_double_register_3')
    translational_error = np.linalg.norm(Hvt[0:3])
    rotational_error = np.linalg.norm(Hvt[3:6])
    trans_error_log = np.append(trans_error_log, [translational_error])
    orientation_error_log = np.append(orientation_error_log, [rotational_error])

    if Etot_prev is not None:
        # Numerical derivative (finite difference)
        dEtot = (Etot - Etot_prev) / dt
        #print(f"t = {t:.2f}, dEtot/dt = {dEtot:.4f}")
    Etot_prev = Etot
    dEtot_log = np.append(dEtot_log, [dEtot], axis=0)

    #qvel = (q - previous_q) / dt
    #previous_q = q.copy()
    '''
    print(f"Hvt (error in current frame): {Hvt}")6.0427208
    print(f"q: {q}")
    print(f"qdot: {qdot}")
    print(f"wrench: {wrench}")
    print(f"Pmax: {Pmax}")
    '''
    jac = robot.jacobian(q)
    grav = robot.gravity(q)
    mass_mat = robot.inertia(q)
    #print("mass_mat:")
    #for row in mass_mat:
    v_act_pos = jac @ qdot

    c_fric = np.transpose(coulomb) * np.sign(qdot)
    v_fric = np.transpose(viscous) * qdot
    c_fric_log = np.append(c_fric_log, [c_fric], axis=0)
    v_fric_log = np.append(v_fric_log, [v_fric], axis=0)
    c_fric_cart = np.linalg.pinv(np.transpose(jac)) @ c_fric
    v_fric_cart = np.linalg.pinv(np.transpose(jac)) @ v_fric
    c_fric_cart_log = np.append(c_fric_cart_log, [c_fric_cart], axis=0)
    v_fric_cart_log = np.append(v_fric_cart_log, [v_fric_cart], axis=0)
    c_fric_sum = np.sum(c_fric)
    v_fric_sum = np.sum(v_fric)
    c_fric_cart_sum = np.sum(c_fric_cart)
    v_fric_cart_sum = np.sum(v_fric_cart)
    c_fric_sum_log = np.append(c_fric_sum_log, [c_fric_sum], axis=0)
    v_fric_sum_log = np.append(v_fric_sum_log, [v_fric_sum], axis=0)
    c_fric_cart_sum_log = np.append(c_fric_cart_sum_log, [c_fric_cart_sum], axis=0)
    v_fric_cart_sum_log = np.append(v_fric_cart_sum_log, [v_fric_cart_sum], axis=0)

    power_coulomb_joint = c_fric * qdot * dt
    power_viscous_joint = v_fric * qdot * dt
    power_c_fric_log = np.append(power_c_fric_log, [power_coulomb_joint], axis=0)
    power_v_fric_log = np.append(power_v_fric_log, [power_viscous_joint], axis=0)
    Frictionpower = power_coulomb_joint + power_viscous_joint
    power_fric_log = np.append(power_fric_log, [Frictionpower], axis=0)
    accumulated_dissipated_energy_per_joint += Frictionpower
    full_power_fric_log = np.append(full_power_fric_log, [accumulated_dissipated_energy_per_joint], axis=0)



    #    print(row)
    #print(f"q: {q}")
    #print(f"Hvt: {Hvt}")
    pvt = -np.array(Hvt[0:3])
    rotvec = -np.array(Hvt[3:6])
    Rvt = R.from_rotvec(rotvec).as_matrix()
    #print(f"Rvt: {Rvt}")
    
    beta_s = 1.0
    beta_s_cart = 1.0
    #lambda_s = 1.0
    #if t > 7.0:
        #Emax = ?
        #Pmax = ?

    delta_p = pvt
    delta_theta = np.array(Hvt[3:6])
    
    pvt_col = pvt.reshape(3, 1)
    #print(f"pvt_col: {pvt_col}")
    #mt1 = vee(skew_symmetric(Go_new @ Rvt))
    #mt2 = vee(skew_symmetric(Gt_new @ Rvt @ pvt_col @ pvt_col.T @ Rvt))
    mt = Ko_new @ delta_theta

    #ft1 = vee(skew_symmetric(Gt_new @ pvt_col)) @ Rvt
    #ft2 = vee(skew_symmetric(Gt_new @ Rvt @ pvt_col @ pvt_col.T @ Rvt))
    ft = Kt_new @ delta_p
    Wt = np.concatenate((mt, ft))
    H0t = pose_to_transformation(current_pose)
    W0 = adjoint_transform(H0t).T @ Wt.T
    pvt_skew = skew_symmetric(pvt_col)
    #Vt = -0.25*np.trace(pvt_col.T @ Gt @ pvt_col) - 0.25 * np.trace(pvt_col.T @ Rvt @ Gt @ Rvt @ pvt_col)
    Rtv = np.transpose(Rvt)
    Vt = -0.25*np.trace(pvt_skew @ Gt @ pvt_skew) - 0.25 * np.trace(pvt_skew @ Rvt @ Gt @ Rtv @ pvt_skew) # måske Rvt Rtv
    #print(f"Vt: {Vt}")
    #print((pvt_col.T @ Gt @ pvt_col).shape)

    ############### MÅSKE ÆNDRE Vo ###############

    #Vo = np.trace(Go @ Rvt)
    #print("Vo", Go @ rotvec)
    #Vo = Go_diag @ rotvec
    Vo = 0.5*np.abs(Go_diag @ delta_theta)
    Vpot = Vt + Vo# + 1.1
    Minv = np.linalg.inv(mass_mat)
    Lambda = np.linalg.inv(jac @ Minv @ np.transpose(jac))
    Tkin_joint = 0.5 * np.transpose(qdot) @ mass_mat @ qdot
    Tkin = 0.5 * np.transpose(v_act) @ Lambda @ v_act
    Tkin_log = np.append(Tkin_log, [Tkin], axis=0)
    ################################################

    #print(f"Vo: {Vo}")
    #Vc = np.trace(Gc @ Rvt @ pvt_col)
    Vt_log = np.append(Vt_log, [Vt], axis=0)
    Vo_log = np.append(Vo_log, [Vo], axis=0)
    Etot = Tkin + Vpot
    if Etot <= Emax:
        lambda_s = 1.0
    else:
        lambda_s = (Emax - Tkin) / Vpot

    Vpot = Vpot * lambda_s
    #Etot = np.clip(Etot, None, Emax)
    #Vpot = np.clip(Vpot, None, Emax)
    Vpot_log = np.append(Vpot_log, [Vpot*lambda_s], axis=0)
    Hvt_log = np.append(Hvt_log, [Hvt], axis=0)
    #print(f"Vpot: {Vpot}")
    #print(Vpot)
    Bo_new = Bo
    Bt_new = Bt
    #B_to = np.block([[Bt_new, Kc],[Kc.T, Bo_new]])
    tau = np.array(tau)
    qdot = np.array(qdot)
    #B_to = np.array(B_to)
    Pc = np.dot(np.transpose(tau - B_full_to @ qdot), qdot)

    scale_rot = 0.3
    scaling = np.diag([3, 3, 3, scale_rot, scale_rot, scale_rot])
    v_act_scaled = scaling @ v_act

    Pc_cart = np.dot(np.transpose(Wt - B_full_to @ v_act_scaled), v_act_scaled) + 1
    W0_log = np.append(W0_log, [W0], axis=0)
    v_act_log = np.append(v_act_log, [v_act], axis=0)
    #print(f"tau: {tau}")
    #print(f"qdot: {qdot}")
    #print(f"Pc: {Pc}")
    #Pcg = gravity.T @ ur5e.d.qvel
 
    #if Pc <= Pmax:
        #beta_s = 1.0
    #else:
        #print(tau)
        #print(qdot)
        #beta_s = (np.transpose(tau) @ qdot - Pmax) / (np.transpose(qdot) @ B_full_to @ qdot)
    if Pc_cart <= Pmax:
        beta_s_cart = 1.0
    else:
        beta_s_cart = (np.transpose(Wt) @ v_act - Pmax) / (np.transpose(v_act) @ B_full_to @ v_act)

    beta_s_cart = np.clip(beta_s_cart, 1, None)
    beta_cart_log = np.append(beta_cart_log, [beta_s_cart], axis=0)
    B_scaled = B_full_to * beta_s
    #Pc = np.dot(tau - B_scaled @ qdot, qdot)
    #Pc = np.clip(Pc, None, Pmax)
    Pc_cart = np.dot(Wt - B_scaled @ v_act_scaled, v_act_scaled) + 1
    Pc_cart = np.clip(Pc_cart, None, Pmax)
    Pc_log = np.append(Pc_log, [Pc], axis=0)
    Pc_cart_log = np.append(Pc_cart_log, [Pc_cart], axis=0)
    beta_log = np.append(beta_log, [beta_s])
    lambda_log = np.append(lambda_log, [lambda_s])
    #Etot = Etot * lambda_s
    Etot = Vpot + Tkin
    #Etot = np.clip(Etot, None, Emax)
    Etot_log = np.append(Etot_log, [Etot])
    #UR_stiffness_log = np.append(UR_stiffness_log, [UR_stiffness], axis=0)
    #UR_damping_log = np.append(UR_damping_log, [UR_damping], axis=0)

    Pcn = np.zeros(6)
    Pcn2 = np.zeros(6)
    #Pc_cart = np.dot(Wt - B_to_new @ v_act, v_act)
    #Pc_cart2 = np.dot(K_full_to @ delta_T_hvt - B_to_new @ v_act, v_act)
    for i in range(6):
        wrench_term =  tau[i]            
        damping_term = B_scaled[i, i] * qdot[i]
        damping_term_cart = B_scaled[i, i] * v_act[i]   
        #print("wrench_term", wrench_term.shape, "damping_term", damping_term.shape)
        Pcn[i] = (wrench_term - damping_term) * qdot[i]# - B_new[i, i] @ ur5e.d.qvel[i]
        Pcn2[i] = (W0[i] - damping_term_cart) * v_act[i]# - B_new[i, i] @ ur5e.d.qvel[i]

    Pcn_cart_log = np.append(Pcn_cart_log, [Pcn2], axis=0)
    Pcn_sum = np.sum(Pcn[0:1])
    Pcn_sum_log = np.append(Pcn_sum_log, [Pcn_sum], axis=0)
    Pcn_cart_sum = np.sum(Pcn2[0:1])
    Pcn_cart_sum_log = np.append(Pcn_cart_sum_log, [Pcn_cart_sum], axis=0)
    Pcn_log = np.append(Pcn_log, [Pcn], axis=0)
    epsilon = 1e-2
    gamma = np.sqrt(2*epsilon)
    Bo_new = Bo * beta_s
    Bt_new = Bt * beta_s
    B_to_new = np.block([[Bt_new, Kc],[Kc.T, Bo_new]])
    tau_nom_log = np.append(tau_nom_log, [tau], axis=0)

    tau_cn = tau# - B_to_new @ qdot
    tau_cn.reshape(6)
    s_n = s_n + (0.2 * Frictionpower)
    H_n = 0.5 * (s_n**2) #+ accumulated_dissipated_energy_per_joint
    ft_sum += ft_error * qdot * 0.05
    H_n = H_n + ft_sum
    H_n_total = np.sum(H_n)
    H_n_total_log = np.append(H_n_total_log, [H_n_total], axis=0)
    #print(H_n)
    H_n_log = np.append(H_n_log, [H_n], axis = 0)
    for i in range(6):
        if H_n[i] > epsilon:# and s_n[i] < s_n_val * 1.05:
            u_n[i] = -tau_cn[i] / s_n[i]
        else:
            u_n[i] = 0#((-tau_cn[i]) / (gamma**2)) * s_n[i]
    
    s_dot = u_n * qdot
    s_n = s_n + s_dot * dt

    u_n_log = np.append(u_n_log, [u_n], axis=0)
    s_n_log = np.append(s_n_log, [s_n], axis=0)
    s_dot_log = np.append(s_dot_log, [s_dot], axis=0)
    qdot_log = np.append(qdot_log, [qdot], axis=0)
    
    W0_cn = W0 - B_to_new @ v_act
    H_n_cart = 0.5 * (s_n_cart**2)
    H_n_cart_total = np.sum(H_n_cart)
    H_n_cart_total_log = np.append(H_n_cart_total_log, [H_n_cart_total], axis=0)
    #u = -tau/s_n_cart
    for i in range(6):
        if H_n_cart[i] > epsilon:
            u_n_cart[i] = -W0_cn[i] / s_n_cart[i]
        else:
            u_n_cart[i] = 0
    if H_n_total > H_n_total_start * 1.05:
        u_n = np.zeros(6)
        #u_n_cart = np.zeros(6)
        #s_n = [0, 0, 0, 0, 0, 0]
    tau_on = -u_n * s_n
    tau_on_log = np.append(tau_on_log, [tau_on], axis=0)
    #u_log = np.append(u_log, [u], axis=0)
    s_dot_cart = u_n_cart * v_act #acc_act
    s_n_cart = s_n_cart + s_dot_cart * dt
    p_dot_on = -u_n_cart * s_n_cart

    H_n_cart_log = np.append(H_n_cart_log, [H_n_cart], axis=0)
    u_n_cart_log = np.append(u_n_cart_log, [u_n_cart], axis=0)
    s_n_cart_log = np.append(s_n_cart_log, [s_n_cart], axis=0)
    s_dot_cart_log = np.append(s_dot_cart_log, [s_dot_cart], axis=0)
    v_act_cart_log = np.append(v_act_cart_log, [v_act], axis=0)
    v_act_pos_log = np.append(v_act_pos_log, [v_act_pos], axis=0)
    tau_on_cart = -u_n_cart * s_n_cart
    tau_on_cart_log = np.append(tau_on_cart_log, [tau_on_cart], axis=0)
    

    
    #print("="*50)
    # Log data
    time_log.append(t)
    #nominal_torque_log = np.append(nominal_torque_log, [torques], axis=0)
    #safe_torque_log = np.append(safe_torque_log, [tau_safe], axis=0)
    t += 0.02
    #tau_zero = [0, 0, 0, 0, 0, 0]  
    for i in range(0,10):
        setattr(setp, f'input_double_register_{i}', 0.0)
    setattr(setp, f'input_double_register_0', Vpot)
    setattr(setp, f'input_double_register_1', beta_s)
    setattr(setp, f'input_double_register_8', lambda_s)
    
    for i in range (2,8):
        setattr(setp, f'input_double_register_{i}', tau_on[i-2])
    #setp.input_double_register_0 = Vpot
    con.send(setp)
    con.send(watchdog)
    
for i in range(0,8):
    setattr(setp, f'input_double_register_{i}', 0.0)
setattr(setp, f'input_double_register_0', Vpot)
setattr(setp, f'input_double_register_1', 1.0)
setattr(setp, f'input_double_register_8', 1.0)
con.send(setp)
con.send(watchdog)
con.send_pause()
con.disconnect()
record_process.send_signal(signal.SIGINT)
try:
    record_process.wait(timeout=5)  # Wait for it to clean up
except subprocess.TimeoutExpired:
    print("record.py did not exit, force killing...")
    record_process.kill()

log_data = {
    'time': time_log,
    'Etot': Etot_log,
    'Vpot': Vpot_log,
    'Tkin': Tkin_log,
    'Pc': Pc_log,
    'Pc_cart': Pc_cart_log,
    'lambda': lambda_log,
    'beta': beta_log,
    'trans_error': trans_error_log,
    'orientation_error': orientation_error_log,
}

# Add selected joint-wise logs (example: only first 3 joints)
for i in range(3):
    log_data[f's_n_j{i+1}'] = s_n_log[:, i]
    log_data[f'u_n_j{i+1}'] = u_n_log[:, i]
    log_data[f's_dot_j{i+1}'] = s_dot_log[:, i]
    log_data[f'tau_on_j{i+1}'] = tau_on_log[:, i]
    log_data[f'qdot_j{i+1}'] = qdot_log[:, i]
    log_data[f'c_fric_j{i+1}'] = c_fric_log[:, i]
    log_data[f'v_fric_j{i+1}'] = v_fric_log[:, i]
    log_data[f'power_fric_j{i+1}'] = power_fric_log[:, i]

# Create DataFrame and save
df = pd.DataFrame(log_data)
df.to_csv(log_filename_local, index=False)
print(f"Logs saved to {log_filename_local}")

time = np.array(time_log)

n_joints = 6
'''
plt.figure(figsize=(8, 5))
plt.plot(time, Pcn_sum_log, 'r-', label='Pcn sum')
plt.axhline(Emax, color='k', linestyle=':', label='Emax')
plt.xlabel('Time (s)')
plt.ylabel('Etot')
plt.title('Total Energy (Etot)')
plt.grid()
plt.legend()
plt.tight_layout()
'''

PlotFTsensor = True
PlotTorque = False
Plot1 = True # Etot, lambda, Pc cart, Pc, beta, tracking error
Plot2 = True # Energy tanks - cartesian
Plot3 = True  # Energy tanks - joint space (bad)
PlotFriction = False
PlotCartesianHn = False # H_n cartesian

if PlotFTsensor:
    wrench_labels = ['Fx (N)', 'Fy (N)', 'Fz (N)', 'Tx (Nm)', 'Ty (Nm)', 'Tz (Nm)']

    plt.figure(figsize=(12, 10))
    for i in range(6):
        plt.subplot(3, 2, i + 1)
        plt.plot(time, ft_error_log[:, i], 'r--', label='F/T')
        plt.xlabel('Time (s)')
        plt.ylabel(wrench_labels[i])
        plt.title(f'External wrench: {wrench_labels[i]}')
        plt.legend()
        plt.grid(True)

    plt.tight_layout()
    plt.suptitle("Force/Torque Sensor Error Over Time", y=1.02)

    plt.figure(figsize=(12, 10))
    for i in range(6):
        plt.subplot(3, 2, i + 1)
        plt.plot(time, wrenchAtTool_log[:, i], 'r--')
        plt.xlabel('Time (s)')
        plt.ylabel(wrench_labels[i])
        plt.title(f'External wrench: {wrench_labels[i]}')
        plt.legend()
        plt.grid(True)

    plt.tight_layout()
    plt.suptitle("Force/Torque Sensor Error Over Time", y=1.02)

    plt.figure(figsize=(10,6))
    plt.plot(time, trans_error_log, 'r-', label='Translational Error ||pvt||')
    plt.plot(time, orientation_error_log, 'b-', label='Rotational Error ||rotvec||')

    plt.xlabel('Time (s)')
    plt.ylabel('Error magnitude')
    plt.title('Pose Error Over Time')
    plt.legend()
    plt.grid()
    plt.tight_layout()


if PlotTorque:
    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, tau_on_log[:,i], 'r--', label = 'tau on')
        #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
        #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
        #plt.plot(time, beta_log[:], 'b-', label = 'beta')
        #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
        #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
        #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
        #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.title('tau on')
        plt.legend()
    plt.tight_layout()


    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, tau_on_cart_log[:,i], 'r--', label = 'tau cart')
        #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
        #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
        #plt.plot(time, beta_log[:], 'b-', label = 'beta')
        #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
        #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
        #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
        #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.title('Cart tau')
        plt.legend()
    plt.tight_layout()

if Plot1:
    #### Plot 1 ####
    # Plot Etot_log


    plt.figure(figsize=(8, 5))
    plt.plot(time, Vpot_log, 'r-')
    #plt.axhline(Emax, color='k', linestyle=':', label='Emax')
    plt.xlabel('Time (s)')
    plt.ylabel('Etot')
    plt.title('Potential Energy')
    plt.grid()
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(8, 5))
    plt.plot(time, Tkin_log, 'r-')
    #plt.axhline(Emax, color='k', linestyle=':', label='Emax')
    plt.xlabel('Time (s)')
    plt.ylabel('Etot')
    plt.title('Kinetic energy')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    
    plt.figure(figsize=(8, 5))
    plt.plot(time, Etot_log, 'r-')
    plt.axhline(Emax, color='k', linestyle=':', label='Emax')
    plt.xlabel('Time (s)')
    plt.ylabel('Etot')
    plt.title('Total Energy (Etot)')
    plt.grid()
    plt.legend()
    plt.tight_layout()

    # Plot lambda_log
    plt.figure(figsize=(8, 5))
    plt.plot(time, lambda_log, 'b-')
    plt.xlabel('Time (s)')
    plt.ylabel('Stiffness scaling Factor')
    plt.title('Lambda')
    plt.grid()
    plt.legend()
    plt.tight_layout()



    # Plot Pc_log
    plt.figure(figsize=(8, 5))
    plt.plot(time, Pc_cart_log, 'g-')
    plt.axhline(Pmax, color='k', linestyle=':', label='Pmax')
    plt.xlabel('Time (s)')
    plt.ylabel('Power (W)')
    plt.title('Power')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    '''
    plt.figure(figsize=(8, 5))
    plt.plot(time, Pc_log, 'g-', label='Pc')
    plt.axhline(Pmax, color='k', linestyle=':', label='Pmax')
    plt.xlabel('Time (s)')
    plt.ylabel('Power (W)')
    plt.title('Power Crossing cartesian (Pc)')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    #plt.show()
    '''

    plt.figure(figsize=(8, 5))
    plt.plot(time, beta_cart_log, 'm-')
    plt.xlabel('Time (s)')
    plt.ylabel('Damping scaling Factor')
    plt.title('Beta')
    plt.grid()
    plt.legend()
    plt.tight_layout()


if Plot2:
    '''
    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, Pcn_cart_log[:,i], 'r--', label = 'Pcn cart')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.legend()
    plt.tight_layout()
    
    plt.figure(figsize=(8, 5))
    plt.plot(time, v_fric_sum_log, 'm-', label='viscous')
    plt.xlabel('Time (s)')
    plt.ylabel('Beta')
    plt.title('Beta scaling factor')
    plt.grid()
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(8, 5))
    plt.plot(time, c_fric_sum_log, 'm-', label='coulomb')
    plt.xlabel('Time (s)')
    plt.ylabel('Beta')
    plt.title('Beta scaling factor')
    plt.grid()
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(8, 5))
    plt.plot(time, v_fric_cart_sum_log, 'm-', label='cart viscous')
    plt.xlabel('Time (s)')
    plt.ylabel('Beta')
    plt.title('Beta scaling factor')
    plt.grid()
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(8, 5))
    plt.plot(time, c_fric_cart_sum_log, 'm-', label='cart coulomb')
    plt.xlabel('Time (s)')
    plt.ylabel('Beta')
    plt.title('Beta scaling factor')
    plt.grid()
    plt.legend()
    plt.tight_layout()

    
    '''


    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, H_n_cart_log[:,i], 'r--', label = 'Hn cart')
        #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
        #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
        #plt.plot(time, beta_log[:], 'b-', label = 'beta')
        #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
        #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
        #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
        #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.legend()
    plt.tight_layout()

    # Plot beta_log
    plt.figure(figsize=(8, 5))
    plt.plot(time, H_n_cart_total_log, 'm-', label='H_n_cart total')
    plt.xlabel('Time (s)')
    plt.ylabel('Beta')
    plt.title('Cartesian Energy')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    #plt.show()

if Plot3:
    plt.figure(figsize=(8, 5))
    plt.plot(time, H_n_total_log, 'm-')
    plt.xlabel('Time (s)')
    plt.ylabel('Beta')
    plt.title('Total Energy in Energy Tank')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    #plt.show()

    joint_labels = [f'Joint {i+1}' for i in range(n_joints)]

    plt.figure(figsize=(12, 10))
    for i in range(n_joints):
        plt.subplot(3, 2, i + 1)
        plt.plot(time, H_n_log[:, i], 'r--')
        plt.xlabel('Time (s)')
        plt.ylabel('Energy [J]')
        plt.title(f'Energy - {joint_labels[i]}')
        plt.legend()
        plt.grid(True)

    plt.tight_layout()
    plt.suptitle('Energy Tank', fontsize=14, y=1.02)
    '''
    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, u_n_log[:,i], 'r--', label = 'u_n')
        #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
        #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
        #plt.plot(time, beta_log[:], 'b-', label = 'beta')
        #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
        #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
        #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
        #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.title('u_n')
        plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, s_dot_log[:,i], 'r--', label = 's dot')
        #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
        #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
        #plt.plot(time, beta_log[:], 'b-', label = 'beta')
        #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
        #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
        #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
        #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.title('s dot')
        plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, s_n_log[:,i], 'r--', label = 's_n')
        #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
        #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
        #plt.plot(time, beta_log[:], 'b-', label = 'beta')
        #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
        #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
        #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
        #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.title('s n')
        plt.legend()
    plt.tight_layout()

    
    plt.figure(figsize=(8, 5))
    plt.plot(time, H_n_total_log, 'm-', label='H_n total')
    plt.xlabel('Time (s)')
    plt.ylabel('Energy [J]')
    plt.title('Energy')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    #plt.show()
    
    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, H_n_log[:,i], 'r--', label = 'H_n')
        #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
        #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
        #plt.plot(time, beta_log[:], 'b-', label = 'beta')
        #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
        #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
        #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
        #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.legend()
    plt.tight_layout()
    '''
    if PlotFriction:
        plt.figure(figsize=(12,10))
        for i in range(n_joints):
            plt.subplot(3,2,i+1)
            plt.plot(time, power_v_fric_log[:,i], 'r--', label = 'viscous') 
            #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
            #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
            #plt.plot(time, beta_log[:], 'b-', label = 'beta')
            #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
            #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
            #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
            #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

            plt.xlabel('Time (s)')
            plt.ylabel('W')
            plt.legend()
        plt.tight_layout()

        plt.figure(figsize=(12,10))
        for i in range(n_joints):
            plt.subplot(3,2,i+1)
            plt.plot(time, power_c_fric_log[:,i], 'r--', label = 'coulomb')
            #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
            #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
            #plt.plot(time, beta_log[:], 'b-', label = 'beta')
            #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
            #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
            #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
            #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

            plt.xlabel('Time (s)')
            plt.ylabel('W')
            plt.legend()
        plt.tight_layout()

        plt.figure(figsize=(12,10))
        for i in range(n_joints):
            plt.subplot(3,2,i+1)
            plt.plot(time, full_power_fric_log[:,i], 'r--', label = 'full friction')
            #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
            #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
            #plt.plot(time, beta_log[:], 'b-', label = 'beta')
            #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
            #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
            #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
            #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

            plt.xlabel('Time (s)')
            plt.ylabel('W')
            plt.legend()
        plt.tight_layout()
    '''

    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, s_n_log[:,i], 'r--', label = 's_n')
        #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
        #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
        #plt.plot(time, beta_log[:], 'b-', label = 'beta')
        #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
        #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
        #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
        #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, u_n_log[:,i], 'r--', label = 'u_n')
        #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
        #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
        #plt.plot(time, beta_log[:], 'b-', label = 'beta')
        #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
        #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
        #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
        #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, s_dot_log[:,i], 'r--', label = 's_dot')
        #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
        #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
        #plt.plot(time, beta_log[:], 'b-', label = 'beta')
        #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
        #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
        #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
        #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.legend()
    plt.tight_layout()
    '''
    if PlotCartesianHn:
        plt.figure(figsize=(12,10))
        for i in range(n_joints):
            plt.subplot(3,2,i+1)
            plt.plot(time, H_n_cart_log[:,i], 'r--', label = 'Hn cart')
            #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
            #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
            #plt.plot(time, beta_log[:], 'b-', label = 'beta')
            #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
            #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
            #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
            #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

            plt.xlabel('Time (s)')
            plt.ylabel('W')
            plt.legend()
        plt.tight_layout()

        # Plot beta_log
        plt.figure(figsize=(8, 5))
        plt.plot(time, H_n_cart_total_log, 'm-', label='H_n_cart total')
        plt.xlabel('Time (s)')
        plt.ylabel('Energy [J]')
        plt.title('Cartesian Energy Tank')
        plt.grid()
        plt.legend()
        plt.tight_layout()

    '''

    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, s_n_cart_log[:,i], 'r--', label = 's_n cart')
        #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
        #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
        #plt.plot(time, beta_log[:], 'b-', label = 'beta')
        #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
        #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
        #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
        #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, u_n_cart_log[:,i], 'r--', label = 'u_n cart')
        #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
        #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
        #plt.plot(time, beta_log[:], 'b-', label = 'beta')
        #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
        #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
        #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
        #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, s_dot_cart_log[:,i], 'r--', label = 's dot cart') 
        #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
        #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
        #plt.plot(time, beta_log[:], 'b-', label = 'beta')
        #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
        #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
        #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
        #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.legend()
    plt.tight_layout()
    '''
plt.show()
