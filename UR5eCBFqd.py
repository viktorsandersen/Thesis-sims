import sys
import numpy as np
sys.path.append("..")
import logging
from cvxopt import matrix, solvers
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import matplotlib.pyplot as plt
#import jax
#import jax.numpy as jnp
from cvxopt import matrix, solvers
import ur_robot
robot = ur_robot.URRobot(ur_robot.RobotType.UR5e)
import threading
import sys
import termios
import tty
from scipy.signal import savgol_filter
from scipy.signal import butter, filtfilt
solvers.options['show_progress'] = False
import pandas as pd
import subprocess
import signal

# Start record.py as a background process
record_process = subprocess.Popen([
    'python3', 'record.py',
    '--frequency', '500',
    '--output', 'withTanh/CBFqd02onepushStiff50RTDE.csv'
])

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
qdot_log = np.zeros((0, 6))
cbf_log = np.zeros((0, 1))
h_dot_log = []
h_dot_test_log = []
# logging.basicConfig(level=logging.INFO)

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
con.send_output_setup(state_names, state_types, frequency=500)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

for i in range(0,10):
        setattr(setp, f'input_double_register_{i}', 0.0)
con.send(setp)


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
    coriolis = [getattr(state, f'output_double_register_{i}') for i in range(36, 42)]
    return mass_mat, coriolis

def moving_average(signal, window_size=5):
    return np.convolve(signal, np.ones(window_size)/window_size, mode='same')

def butter_lowpass_filter(data, cutoff, fs, order=4):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y
fs = 1 / 0.02  # Sampling frequency (Hz)
cutoff = 2.0   # Cutoff frequency (Hz)

t = 0.0
idx = 0
qd_max_s = 0.2
#qd_max = jnp.array([qd_max_s, 0, 0, 0, 0, 0])
alpha = 4000
move_completed = True
threading.Thread(target=check_for_q, daemon=True).start()
while keep_running:
    state = con.receive()
    if state is None:
        break
    
    Hvt = [getattr(state, f'output_double_register_{i}') for i in range(42,48)]
    q = [getattr(state, f'output_double_register_{i}') for i in range(36,42)]
    #q = np.array(q)
    q = np.array(q).reshape(6, 1)
    qdot = [getattr(state, f'output_double_register_{i}') for i in range(30,36)]
    qdot = np.array(qdot).reshape(6, 1)
    qdot_log = np.append(qdot_log, [qdot.flatten()], axis=0)
    v_act = [getattr(state, f'output_double_register_{i}') for i in range(24,30)]
    tau = [getattr(state, f'output_double_register_{i}') for i in range(18,24)]
    #timecounter = getattr(state, f'output_double_register_10')
    #tc_log = np.append(tc_log, [timecounter], axis=0)
    #UR_stiffness = [getattr(state, f'output_double_register_{i}') for i in range(12,18)]
    #UR_damping = [getattr(state, f'output_double_register_{i}') for i in range(6,12)]
    current_pose = [getattr(state, f'output_double_register_{i}') for i in range(6,12)]
    current_pose = np.array(current_pose).reshape(6, 1)
    jac = robot.jacobian(q)
    grav = robot.gravity(q)
    M_full = robot.inertia(q)
    C_full = robot.coriolis(q, qdot)
    #print("Coriolis", C_full)
    C = C_full @ qdot
    #print("Coriolis", C)
    n = 6
    x = np.hstack((q, qdot))
    B_scalar = qd_max_s - qdot[0]
    cbf_log = np.append(cbf_log, [B_scalar], axis=0)
    B = np.array([0, 0, 0, 0, 0, 0, B_scalar[0], 0, 0, 0, 0, 0])
    dB_matrix = np.hstack((np.zeros((n,n)), -np.eye(n))) #
    dB = np.hstack((np.zeros(n), np.array([-1, 0, 0, 0, 0 ,0]))).reshape(12,1)
    #print("M", M_full.shape, "C", C_full.shape, "linalg", np.linalg.solve(M_full, -C_full).shape)
    f = np.hstack((qdot, np.linalg.solve(M_full, -C))).reshape(12,1)
    g = np.vstack((np.zeros((6,6)), np.linalg.inv(M_full)))
    LfB = np.transpose(dB) @ f
    LgB = np.transpose(dB) @ g
    #print("LgB", LgB.shape, "dB", dB.shape, "g", g.shape, "dB", dB.shape)
    h_val = B

    u_nom = np.array(tau)
    u_nom = u_nom.reshape(-1, 1)
    #print("LfB", LfB.shape, "LgB", LgB.shape, "u_nom", u_nom.shape, "B", B.shape)
    #print(h_dot_test, "\n")
    A_cbf = -LgB
    b_cbf = (LfB + alpha * B_scalar.reshape(-1,1)).flatten() # h - 0.002
    #print("LfB", LfB, "LgB", LgB, "u_nom", "LgB*u_nom", LgB @ u_nom)
    h_dot_test = LfB + LgB @ u_nom
    h_dot_test_log.append(h_dot_test[0])

    # Numerical h\dot finite differences
    if len(cbf_log) > 1:
        h_dot = (cbf_log[-1] - cbf_log[-2]) / 0.02 
        h_dot_log.append(h_dot[0])
    else:
        h_dot_log.append(0.0)

    #n = u_nom.shape[0]
    P = matrix(np.eye(n))
    q = matrix(-u_nom, tc='d')
    G = matrix(A_cbf, tc='d')
    #b_cbf = np.asarray(b_cbf, dtype=np.float64).flatten()
    h_qp = matrix(b_cbf)

    sol = solvers.qp(P,q,G,h_qp)
    if sol['status'] != 'optimal':
        print("Stop")
        con.send_pause()
        con.disconnect()

    tau_safe = np.array(sol['x']).flatten()
    tau_safe = [tau_safe[0], tau_safe[1], tau_safe[2], 0, 0, 0]
    for i in range(2,8):
        setattr(setp, f'input_double_register_{i}', tau_safe[i-2])
    #setp.input_double_register_0 = torques[0]
    con.send(setp)
    con.send(watchdog)
    #print(f"Target Command: {torques}")
    
    time_log.append(t)
    nominal_torque_log = np.append(nominal_torque_log, [tau], axis=0)
    safe_torque_log = np.append(safe_torque_log, [tau_safe], axis=0)
    t += 0.02
tau_zero = [0, 0, 0, 0, 0, 0]  
for i in range(6):
    setattr(setp, f'input_double_register_{i}', tau_zero[i])
    #setp.input_double_register_0 = torques[0]
    record_process.send_signal(signal.SIGINT)
    try:
        record_process.wait(timeout=5)  # Wait for it to clean up
    except subprocess.TimeoutExpired:
        print("record.py did not exit, force killing...")
        record_process.kill()
#con.send(setp)
#con.send(watchdog)
for i in range(0,8):
    setattr(setp, f'input_double_register_{i}', 0.0)
con.send(setp)
con.send(watchdog)
con.send_pause()
con.disconnect()

log_data = {
    'time': time_log,
    'cbf': cbf_log.flatten(),
    'h_dot': h_dot_log,
    'h_dot_test': [val[0] if isinstance(val, np.ndarray) else val for val in h_dot_test_log],
}

# Add joint-wise data
for i in range(3):  # assuming first 3 joints are relevant
    log_data[f'nominal_torque_j{i+1}'] = nominal_torque_log[:, i]
    log_data[f'safe_torque_j{i+1}'] = safe_torque_log[:, i]
    log_data[f'qdot_j{i+1}'] = qdot_log[:, i]

# Save to CSV
df = pd.DataFrame(log_data)
df.to_csv('CBFqd02nopush.csv', index=False)
print("Logs saved to 'withTanh/CBFqd02onepushStiff50.csv'")

time = np.array(time_log)
n_joints = 3

h_dot_log_smooth = moving_average(h_dot_log, window_size=10)
h_dot_log_butter = butter_lowpass_filter(h_dot_log, cutoff=cutoff, fs=fs)
h_dot_log_savgol = savgol_filter(h_dot_log, window_length=15, polyorder=3)

plt.figure(figsize=(12,10))
for i in range(n_joints):
    plt.subplot(3,2,i+1)
    plt.plot(time, nominal_torque_log[:,i], 'r--', label = 'Nominal torque')
    #plt.plot(time, safe_torque_log[:,i], 'b-', label = 'Safe torque')
    #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
    #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

    plt.xlabel('TIme (s)')
    plt.ylabel(f'Joint {i+1} torque')
    plt.legend()
    plt.title(f'Joint {i+1} torque comparison')
plt.tight_layout()


plt.figure(figsize=(12,10))
for i in range(n_joints):
    plt.subplot(3,2,i+1)
    plt.plot(time, qdot_log[:,i], 'b-', label = 'Safe torque')
    #plt.axhline(MIN_TORQUE[i], color='k', linestyle=':', label='tau min')
    #plt.axhline(MAX_TORQUE[i], color='k', linestyle='--', label='tau max')

    plt.xlabel('TIme (s)')
    plt.ylabel(f'Joint {i+1} torque')
    plt.legend()
    plt.title(f'qdot')
plt.tight_layout()


# plot cbf_log
plt.figure(figsize=(12,10))
plt.subplot(3,1,1)
plt.plot(time, cbf_log, 'b-', label = 'barrier value')
plt.axhline(0, color='k', linestyle=':', label='tau min')
plt.axhline(0.002, color='k', linestyle='--', label='tau max')
plt.xlabel('TIme (s)')
plt.ylabel(f'CBF')
plt.legend()
plt.title(f'CBF')
plt.tight_layout()

plt.figure(figsize=(12,10))
plt.subplot(3,1,1)
#plt.plot(time, h_dot_log, 'b-', label = 'numerical diff h')
#plt.plot(time, h_dot_log_smooth, 'r--', label = 'numerical diff h smoothed')
plt.plot(time, h_dot_log_butter, 'g-', label = 'numerical diff h butter')
#plt.plot(time, h_dot_test_log, 'r-', label = 'diff h')
#plt.plot(time, h_dot_log_savgol, 'b--', label = 'numerical diff h savgol')
#plt.axhline(0, color='k', linestyle=':', label='tau min')
#plt.axhline(0.002, color='k', linestyle='--', label='tau max')
plt.xlabel('TIme (s)')
plt.ylabel(f'CBF')
plt.legend()
plt.title(f'CBF')
plt.tight_layout()

cbf_violation = np.minimum(cbf_log, 0)  # Only negative values (<= 0), else 0

plt.figure(figsize=(12,6))
plt.plot(time, cbf_violation, 'r-', label='CBF violation (negative values)')
plt.axhline(0, color='k', linestyle='--', label='Zero threshold')
plt.xlabel('Time (s)')
plt.ylabel('CBF value')
plt.title('CBF Violations Over Time (How Negative)')
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.show()