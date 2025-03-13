import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from numpy.linalg import pinv

# Load Data and Compute Torque
data = pd.read_csv('data/robot_data_ur5_full_speed_wrench_new.csv')

# Extract actual current values (columns for target_current_0 to target_current_5)
actual_current = data.iloc[:, 43:49].values  # This gives us the columns 44 to 49 (0-indexed)

# Check the shape of actual_current to ensure it's (5000, 6)
print(f'Shape of actual_current: {actual_current.shape}')

pos = pd.read_csv('pos.csv', header=None).values
vel = pd.read_csv('vel.csv', header=None).values
acc = pd.read_csv('acc.csv', header=None).values
print(actual_current.shape)
'''
print(actual_current.shape)
gear_ratio = 101
torque_constants = np.array([0.07695, 0.07695, 0.07695, 0.098322, 0.098322, 0.098322])

torque = np.zeros_like(actual_current)
for i in range(6):
    torque[:, i] = gear_ratio * torque_constants[i] * actual_current[:, i]

# Segment the Data into Phases and Average Them
phase_length = 5000
total_samples = torque.shape[0]
num_phases = total_samples // phase_length

avg_phase_torque = np.zeros((phase_length, 6))

for joint in range(6):
    torque_reshaped = torque[:num_phases * phase_length, joint].reshape(phase_length, num_phases)
    avg_phase_torque[:, joint] = np.mean(torque_reshaped, axis=1)

Fs = 1000
t_phase = np.arange(phase_length) / Fs

# Plot the torque for each joint and phase
fig, axes = plt.subplots(3, 2, figsize=(10, 8))
for joint in range(6):
    ax = axes[joint // 2, joint % 2]
    for phase in range(num_phases):
        start_idx = phase * phase_length
        end_idx = (phase + 1) * phase_length
        ax.plot(np.arange(phase_length), torque[start_idx:end_idx, joint])
    ax.set_xlabel('Samples in Phase')
    ax.set_ylabel('Torque (Nm)')
    ax.grid(True)
    ax.set_xlim([1000, 2000])
    ax.legend([f'Phase {i + 1}' for i in range(num_phases)])
    ax.set_title(f'Joint {joint + 1} Torque')

plt.tight_layout()
plt.show()

# Plot average torque
fig, axes = plt.subplots(6, 1, figsize=(10, 12))
for joint in range(6):
    axes[joint].plot(t_phase, avg_phase_torque[:, joint], 'b-', linewidth=1)
    axes[joint].set_xlabel('Time (s)')
    axes[joint].set_ylabel(f'Torque (Nm) - Joint {joint + 1}')
    axes[joint].set_title(f'Joint {joint + 1}: Averaged Torque')
    axes[joint].legend(['Averaged Phase'])

plt.tight_layout()
plt.show()

# Design and Apply the Butterworth Filter
# Commented out since no filter is applied in the original code
# cutoff = 10  # Cutoff frequency in Hz
# order = 4  # Filter order
# b, a = butter(order, cutoff / (Fs / 2))  # Normalize cutoff frequency
# avg_phase_torque_filtered = np.zeros_like(avg_phase_torque)
# for joint in range(6):
#     avg_phase_torque_filtered[:, joint] = filtfilt(b, a, avg_phase_torque[:, joint])

# Load Robot Model and Prepare Dynamics Inputs
# Replace this with a library like `robotics-toolbox` or similar
# Assuming `robot_UR5e` is defined in the model
# For simplicity, using mock data for `q`, `dq`, `ddq`
q = np.random.rand(6, 100)  # Example joint positions
dq = np.random.rand(6, 100)  # Example joint velocities
ddq = np.random.rand(6, 100)  # Example joint accelerations

num_samples = q.shape[1]

grav_all = np.zeros((6, num_samples))
tau_model = np.zeros((6, num_samples))
jac_all = [None] * num_samples
inertia_all = [None] * 6
vel_prod_all = np.zeros((6, num_samples))

# Mock dynamics calculation for robot (assuming a function like UR5e dynamics)
# Here we simply use placeholder values for grav_all, jac_all, inertia_all, etc.
for i in range(num_samples):
    q_i = q[:, i]
    dq_i = dq[:, i]
    ddq_i = ddq[:, i]
    
    grav_all[:, i] = np.random.rand(6)  # Mock gravitational torques
    jac_all[i] = np.random.rand(6, 6)  # Mock Jacobian
    inertia_all[i] = np.random.rand(6, 6)  # Mock inertia matrix
    vel_prod_all[:, i] = np.random.rand(6)  # Mock velocity product
    
    tau_model[:, i] = inertia_all[i] @ ddq_i + vel_prod_all[:, i] + grav_all[:, i]

# Plot the torque model and averaged torque
fig, axes = plt.subplots(6, 1, figsize=(10, 12))
for joint in range(6):
    axes[joint].plot(t_phase, tau_model[joint, :], linewidth=1.5, label='Model Torque')
    axes[joint].plot(t_phase, avg_phase_torque[:, joint], linewidth=1.5, label='Averaged Torque')
    axes[joint].set_xlabel('Time (s)')
    axes[joint].set_ylabel('Torque (Nm)')
    axes[joint].set_title(f'Joint {joint + 1} Torque')
    axes[joint].legend()

plt.tight_layout()
plt.show()

# Regressor matrix
tau_py = pd.read_csv('torques.csv', header=None).values.T
tau_fric = avg_phase_torque.T - tau_model  # Measurement - model

Y = np.zeros((6 * num_samples, 12))
for k in range(num_samples):
    for i in range(6):
        row_idx = (k * 6) + i
        Y[row_idx, i] = np.sign(dq[i, k])  # Coulomb
        Y[row_idx, 6 + i] = dq[i, k]  # Viscous

tau_friction_vec = tau_fric.flatten()

# Solve for the friction coefficients
pi = pinv(Y) @ tau_friction_vec

# Extract the friction coefficients
f_coulomb = pi[:6]
f_viscous = pi[6:12]

print('Estimated Coulomb friction coefficients:')
print(f_coulomb)

print('Estimated Viscous friction coefficients:')
print(f_viscous)

# Estimate the friction torque
tau_friction_est = Y @ pi
tau_friction_est = tau_friction_est.reshape(6, num_samples)

tau_full = tau_model + tau_friction_est

# Plot full torque vs measured torque
fig, axes = plt.subplots(6, 1, figsize=(10, 12))
for joint in range(6):
    axes[joint].plot(t_phase, tau_full[joint, :], linewidth=1.5, label='Full Model')
    axes[joint].plot(t_phase, avg_phase_torque[:, joint], linewidth=1.5, label='Measured Torque')
    axes[joint].set_xlabel('Time (s)')
    axes[joint].set_ylabel('Torque (Nm)')
    axes[joint].set_title(f'Joint {joint + 1} Full Torque vs Measured Torque')
    axes[joint].legend()

plt.tight_layout()
plt.show()

# Autocovariance
fig, axes = plt.subplots(6, 2, figsize=(14, 12))
for joint in range(6):
    # Plot torque
    ax = axes[joint, 0]
    ax.plot(tau_full[joint, :], 'b-', linewidth=1.5, label='Tau model')
    ax.plot(avg_phase_torque[:, joint], 'r--', linewidth=1.5, label='Averaged Measured')
    ax.set_xlabel(f'q{joint + 1} [rad/s]')
    ax.set_ylabel('Torque [Nm]')
    ax.set_title(f'Joint {joint + 1} Torque')
    ax.legend()
    
    # Compute autocovariance for friction
    tau_fric_centered = tau_fric[joint, :] - np.mean(tau_fric[joint, :])
    autocovariance, lags = np.correlate(tau_fric_centered, tau_fric_centered, mode='full')
    
    # Plot autocovariance
    ax = axes[joint, 1]
    ax.plot(lags, autocovariance, 'b-', linewidth=1.5)
    ax.set_xlabel('Lag')
    ax.set_ylabel('Autocovariance')
    ax.set_title(f'Autocovariance eτ{joint + 1}')
    ax.grid(True)

plt.tight_layout()
plt.show()
'''