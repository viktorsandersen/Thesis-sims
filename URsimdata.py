import sys
import numpy as np
sys.path.append("..")
import logging
from cvxopt import matrix, solvers
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import matplotlib.pyplot as plt

import threading
import sys
import termios
import tty

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
# logging
time_log = []
nominal_torque_log = np.zeros((0, 6))
safe_torque_log = np.zeros((0, 6))
# logging.basicConfig(level=logging.INFO)

ROBOT_HOST = "172.17.0.2" #URsim
#ROBOT_HOST = "192.168.0.100"
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



#setp.input_double_register_0 = 0


# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
watchdog.input_int_register_0 = 0


def setp_to_list(sp):
    sp_list = []
    for i in range(0, 6):
        sp_list.append(sp.__dict__["input_double_register_%i" % i])
    return sp_list


def list_to_setp(sp, list):
    for i in range(0, 6):
        sp.__dict__["input_double_register_%i" % i] = list[i]
    return sp


if not con.send_start():
    sys.exit()

def unpack_mass_matrix_and_coriolis(state):
    mass_flat = [getattr(state, f'output_double_register_{i}') for i in range(0, 36)]
    mass_mat = [mass_flat[i:i+6] for i in range(0, 36, 6)]
    #coriolis = [getattr(state, f'output_double_register_{i}') for i in range(36, 42)]
    return mass_mat#, coriolis

t = 0.0
idx = 0
DAMPING = np.eye(6)
DAMPING[0, 0] *= 100
DAMPING[1, 1] *= 100
DAMPING[2, 2] *= 100
DAMPING[3, 3] *= 20
DAMPING[4, 4] *= 20
DAMPING[5, 5] *= 20
move_completed = True
threading.Thread(target=check_for_q, daemon=True).start()
while keep_running:
    state = con.receive()
    if state is None:
        break

    # Read target_command from registers 0-5
    #target_command = [getattr(state, f'output_double_register_{i}') for i in range(6)]
    #print(f"Target Command: {target_command}")

    # Read mass matrix (6x6 = 36 values starting from register 6)
    #mass_matrix_flat = [getattr(state, f'output_double_register_{i}') for i in range(6, 42)]
    #mass_matrix = [mass_matrix_flat[i:i+6] for i in range(0, 36, 6)]
    #print("Mass Matrix:")
    #for row in mass_matrix:
        #print(row)

    # Read coriolis vector (6 values from register 42 to 47)
    #coriolis_vector = [getattr(state, f'output_double_register_{i}') for i in range(42, 48)]
    #print(f"Coriolis Vector: {coriolis_vector}")
    Pc = [getattr(state, f'output_double_register_47')]
    joint_vel = [getattr(state, f'output_double_register_{i}') for i in range(42,47)]
    joint_vel = np.array(joint_vel)
    joint_vel = np.append(joint_vel, 0.0)
    wrench = [getattr(state, f'output_double_register_{i}') for i in range(36,42)]
    wrench = np.array(wrench)
    #print(torques.shape)

    #setp.input_double_register_0 = torques[0]
    print(f"Pc: {Pc}")
    print(f"Joint vel: {joint_vel}")
    print(f"wrench: {wrench}")
    jacobian = unpack_mass_matrix_and_coriolis(state)
    print("jacobian:")
    for row in jacobian:
        print(row)
    #print(f"Coriolis Vector: {coriolis}")
    Pc2 = np.dot(np.transpose((jacobian) @ wrench) - (DAMPING @ joint_vel), joint_vel)
    print(f"Pc2: {Pc2}")
    print("="*50)
    # Log data
    time_log.append(t)
    t += 0.02
tau_zero = [0, 0, 0, 0, 0, 0]  
for i in range(6):
    setattr(setp, f'input_double_register_{i}', tau_zero[i])
    #setp.input_double_register_0 = torques[0]

con.send(setp)
con.send(watchdog)
con.send_pause()
con.disconnect()

time = np.array(time_log)
n_joints = 6

plt.figure(figsize=(12,10))
for i in range(n_joints):
    plt.subplot(3,2,i+1)
    plt.plot(time, nominal_torque_log[:,i], 'r--', label = 'Nominal torque')
    plt.plot(time, safe_torque_log[:,i], 'b-', label = 'Safe torque')
    plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
    plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

    plt.xlabel('TIme (s)')
    plt.ylabel(f'Joint {i+1} torque')
    plt.legend()
    plt.title(f'Joint {i+1} torque comparison')
plt.tight_layout()
plt.show()