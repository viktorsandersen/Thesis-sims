import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV log
df = pd.read_csv("robot_control_logs.csv")

# Plot settings
n_joints = 3
time = df['time']

# Torque comparison plots
plt.figure(figsize=(12, 10))
for i in range(n_joints):
    plt.subplot(3, 1, i+1)
    plt.plot(time, df[f'nominal_torque_j{i+1}'], 'r--', label='Nominal Torque')
    plt.plot(time, df[f'safe_torque_j{i+1}'], 'b-', label='Safe Torque')
    plt.xlabel('Time (s)')
    plt.ylabel(f'Joint {i+1} Torque')
    plt.title(f'Joint {i+1} Torque Comparison')
    plt.legend()
plt.tight_layout()

# qdot plot
plt.figure(figsize=(12, 10))
for i in range(n_joints):
    plt.subplot(3, 1, i+1)
    plt.plot(time, df[f'qdot_j{i+1}'], 'g-', label='qdot')
    plt.xlabel('Time (s)')
    plt.ylabel(f'Joint {i+1} Velocity')
    plt.title(f'Joint {i+1} Velocity')
    plt.legend()
plt.tight_layout()

# CBF value plot
plt.figure(figsize=(10, 4))
plt.plot(time, df['cbf'], label='CBF value')
plt.axhline(0, color='k', linestyle='--', label='Barrier = 0')
plt.xlabel('Time (s)')
plt.ylabel('CBF')
plt.title('Control Barrier Function Over Time')
plt.legend()
plt.tight_layout()

# CBF derivative comparison
plt.figure(figsize=(10, 4))
plt.plot(time, df['h_dot'], label='Numerical h_dot')
plt.plot(time, df['h_dot_test'], label='Analytical h_dot')
plt.axhline(0, color='k', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('h_dot')
plt.title('CBF Derivatives Comparison')
plt.legend()
plt.tight_layout()

plt.show()
