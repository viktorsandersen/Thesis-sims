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
np.set_printoptions(precision=8, suppress=True)
import ur_robot
robot = ur_robot.URRobot(ur_robot.RobotType.UR5e)
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
UR_stiffness_log = np.zeros((0, 6))
UR_damping_log = np.zeros((0, 6))
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
Pmax = 5.0
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
coulomb = np.array([4.76, 4.73, 3.36, 1.36, 1.5, 1.51])
viscous = np.array([9.79, 6.0, 4.5, 1.5, 1.4, 3.2])
#coulomb = np.array([9.1, 4.7, 3.3, 1.25, 1.65, 1.6])
#viscous = np.array([14.3, 5.6, 4.3, 1.35, 1.55, 3.35])
accumulated_dissipated_energy_per_joint = 0
H_n_start = 0.5 * (s_n_start**2)
threading.Thread(target=check_for_q, daemon=True).start()
print("Press 'q' to quit")
while keep_running:
    state = con.receive()
    if state is None:
        break
    #mass_mat, vel = unpack_mass_matrix_and_coriolis(state)
    #print(f"Joint vel: {vel}")

    Hvt = [getattr(state, f'output_double_register_{i}') for i in range(42,48)]
    #q = [getattr(state, f'output_double_register_{i}') for i in range(36,42)]
    wrenchAtTool = [getattr(state, f'output_double_register_{i}') for i in range(36,42)]
    wrenchAtTool_log = np.append(wrenchAtTool_log, [wrenchAtTool], axis=0)
    #q = np.array(q).reshape(6, 1)
    #qdot = [getattr(state, f'output_double_register_{i}') for i in range(30,36)]
    ft_error = [getattr(state, f'output_double_register_{i}') for i in range(30,36)]
    ft_error_log = np.append(ft_error_log, [ft_error], axis=0)
    #v_act = [getattr(state, f'output_double_register_{i}') for i in range(24,30)]
    ftBaseFrame = [getattr(state, f'output_double_register_{i}') for i in range(24,30)]
    ftBaseFrame_log = np.append(ftBaseFrame_log, [ftBaseFrame], axis=0)
    translational_error = np.linalg.norm(Hvt[0:3])
    trans_error_log = np.append(trans_error_log, [translational_error])
    rotational_error = np.linalg.norm(Hvt[3:6])
    orientation_error_log = np.append(orientation_error_log, [rotational_error])

    
    #print("="*50)
    # Log data
    time_log.append(t)
    #nominal_torque_log = np.append(nominal_torque_log, [torques], axis=0)
    #safe_torque_log = np.append(safe_torque_log, [tau_safe], axis=0)
    t += 0.02
    
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
Plot1 = False # Etot, lambda, Pc cart, Pc, beta, tracking error
Plot2 = False # Pcn cart, Pcn, H_n, tau_on, s_n, s_dot
Plot3 = False  # Energy tanks 
PlotFriction = False
PlotCartesianHn = False # H_n cartesian

if PlotFTsensor:
    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, ft_error_log[:,i], 'r--', label = 'ft error')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.title('ft error')
        plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, wrenchAtTool_log[:,i], 'r--', label = 'wrench at tool')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.title('FT at tool')
        plt.legend()
    plt.tight_layout()

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
    plt.plot(time, Vpot_log, 'r-', label='Vpot')
    #plt.axhline(Emax, color='k', linestyle=':', label='Emax')
    plt.xlabel('Time (s)')
    plt.ylabel('Etot')
    plt.title('Potential Energy')
    plt.grid()
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(8, 5))
    plt.plot(time, Tkin_log, 'r-', label='Tkin')
    #plt.axhline(Emax, color='k', linestyle=':', label='Emax')
    plt.xlabel('Time (s)')
    plt.ylabel('Etot')
    plt.title('Kinetic energy')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    
    plt.figure(figsize=(8, 5))
    plt.plot(time, Etot_log, 'r-', label='Etot')
    plt.axhline(Emax, color='k', linestyle=':', label='Emax')
    plt.xlabel('Time (s)')
    plt.ylabel('Etot')
    plt.title('Total Energy (Etot)')
    plt.grid()
    plt.legend()
    plt.tight_layout()

    # Plot lambda_log
    plt.figure(figsize=(8, 5))
    plt.plot(time, lambda_log, 'b-', label='Lambda scaling')
    plt.xlabel('Time (s)')
    plt.ylabel('Lambda')
    plt.title('Lambda scaling factor')
    plt.grid()
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, UR_stiffness_log[:,i], 'r--', label = 'stiffy')
        #plt.plot(time, lambda_log[:], 'b-', label = 'Lambda')
        #plt.plot(time, Pc_log[:], 'b-', label = 'Pc')
        #plt.plot(time, beta_log[:], 'b-', label = 'beta')
        #plt.plot(time, trans_error[:,i], 'b-', label = 'Safe torque')
        #plt.plot(time, orientation_error[:,i], 'b-', label = 'Safe torque')
        #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
        #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.title('stiffy')
        plt.legend()
    plt.tight_layout()


    # Plot Pc_log
    plt.figure(figsize=(8, 5))
    plt.plot(time, Pc_cart_log, 'g-', label='Pc cart')
    plt.axhline(Pmax, color='k', linestyle=':', label='Pmax')
    plt.xlabel('Time (s)')
    plt.ylabel('Power (W)')
    plt.title('Power Crossing cartesian (Pc)')
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
    plt.plot(time, beta_cart_log, 'm-', label='Beta cart scaling')
    plt.xlabel('Time (s)')
    plt.ylabel('Beta')
    plt.title('Beta scaling factor')
    plt.grid()
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(10,6))
    plt.plot(time, trans_error_log, 'r-', label='Translational Error ||pvt||')
    plt.plot(time, orientation_error_log, 'b-', label='Rotational Error ||rotvec||')

    plt.xlabel('Time (s)')
    plt.ylabel('Error magnitude')
    plt.title('Pose Error Over Time')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    #plt.show()

if Plot2:
    plt.figure(figsize=(12,10))
    for i in range(n_joints):
        plt.subplot(3,2,i+1)
        plt.plot(time, Pcn_cart_log[:,i], 'r--', label = 'Pcn cart')

        plt.xlabel('Time (s)')
        plt.ylabel('W')
        plt.legend()
    plt.tight_layout()
    
    '''
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
    
    plt.figure(figsize=(8, 5))
    plt.plot(time, H_n_total_log, 'm-', label='H_n total')
    plt.xlabel('Time (s)')
    plt.ylabel('Beta')
    plt.title('Beta scaling factor')
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
    plt.title('Beta scaling factor')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()

if Plot3:
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
        plt.ylabel('Beta')
        plt.title('Beta scaling factor')
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
