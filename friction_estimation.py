#!/usr/bin/env python3
"""
This script reads a space‐separated CSV file containing UR5 robot data,
converts columns 1 to 7 (assumed to be q₀ to q₆) to numeric data,
and then plots them.
"""

import pandas as pd
import matplotlib.pyplot as plt

def plot_joints():
    # Specify the path to your CSV file.
    file_path = 'data/robot_data_ur5_full_speed_wrench_first_5k.csv'
    
    # Read the space-separated file.
    # low_memory=False avoids dtype warnings on large files.
    df = pd.read_csv(file_path, sep=r'\s+', header=None, low_memory=False)
    
    # Convert columns 1 to 7 (i.e., q₀ to q₆) to numeric.
    # (Assumes that column 0 is timestamp or something else.)
    q_columns = df.iloc[:, 7:12].apply(pd.to_numeric, errors='coerce')
    
    # Convert the index to a numpy array to avoid multidimensional indexing issues.
    x = q_columns.index.to_numpy()
    
    # Create the plot.
    plt.figure(figsize=(12, 8))
    
    # Loop through each of the 7 q columns.
    for i in range(q_columns.shape[1]):
        # Convert the Series to a numpy array.
        y = q_columns.iloc[:, i].to_numpy()
        plt.plot(x, y, label=f'q_{i}')
    
    plt.xlabel('Entry Index')
    plt.ylabel('qd value')
    plt.title('Plot of q₀ to q₆ (Columns 1 to 7)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    
    # Show the plot.
    plt.show()


def plot_last_torques():
    file_path = 'data/robot_data_ur5_full_speed_wrench_first_5k.csv'
    
    # Read the CSV file using a regular expression for space separation.
    # Setting low_memory=False to avoid dtype warnings.
    df = pd.read_csv(file_path, sep=r'\s+', header=None, low_memory=False)
    
    # Extract the last 6 columns
    last_six = df.iloc[:, -2:]
    
    # Create a plot
    plt.figure(figsize=(10, 6))
    
    # Loop over the last 6 columns and plot each.
    for i, col in enumerate(last_six.columns):
        # Convert each column to numeric (coerce errors)
        y = pd.to_numeric(last_six[col], errors='coerce').to_numpy()
        x = last_six.index.to_numpy()
        plt.plot(x, y, label=f'Column {col}')
    
    plt.xlabel('Entry Index')
    plt.ylabel('Value')
    plt.title('Plot of the Last 6 Columns from CSV')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_last_torque():
    # Update this path to your CSV file.
    file_path = 'data/robot_data_ur5_full_speed_wrench_first_5k.csv'
    
    # Read the CSV file with space as delimiter.
    # low_memory=False avoids dtype warnings on large files.
    df = pd.read_csv(file_path, sep=r'\s+', header=None, low_memory=False)
    
    # Extract the last column and convert to numeric.
    last_col = pd.to_numeric(df.iloc[:, -4], errors='coerce')
    
    # Convert the index to a NumPy array for plotting.
    x = last_col.index.to_numpy()
    y = last_col.to_numpy()
    
    # Create the plot.
    plt.figure(figsize=(10, 6))
    plt.plot(x, y, label='Last Column', color='blue')
    plt.xlabel('Entry Index')
    plt.ylabel('Value')
    plt.title('Plot of the Last Column from CSV')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    plot_last_torque()
