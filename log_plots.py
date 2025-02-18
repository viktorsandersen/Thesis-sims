import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import math as m
from spatialmath import UnitQuaternion as uq

######################
# import csv file 
######################
# Import robot data
filename = 'log_data_red_drawing.csv'
df_rob = pd.read_csv(filename)
# import trajectory data from txt file
df_traj = pd.read_csv('trajectory.txt', sep=", ", header=None)
# name df_traj columns
df_traj.columns = ['x', 'y', 'z', 'Qw', 'Qx', 'Qy', 'Qz']

# array of times when each data point was recorded with 0.002s intervals
time = np.arange(0, len(df_rob)*0.002, 0.002)

# print the names of the columns
#print("Robot data columns:")
#print(df_rob.columns)
#print("Trajectory data columns:")
#print(df_traj.columns)

######################
# Function to plot data
######################

def plot_position(df_rob, df_traj):
    # set figsize
    plt.figure(figsize=(12,8))
    plt.grid()
    # plot the trajectory
    plt.plot(df_traj['x'], df_traj['y'], 'b', label='Trajectory', linewidth=3)
    # plot the robot position
    plt.plot(df_rob['actual_position_0'], df_rob['actual_position_1'], 'r', label='Robot position', linewidth=1)
    # set labels
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    # set legend
    plt.legend()
    # show the plot
    plt.savefig('data/'+filename[:-4]+'_position.pdf')
    # plt.show()

def plot_orientation(df_rob, df_traj):
    # turn orientation into unit quaternions
    # make numpy array of the robot orientation in rpy xyz 
    temp = np.array([df_rob['actual_position_3'], df_rob['actual_position_4'], df_rob['actual_position_5']])
    # make numpy array of uq.RPY with length of temp
    quat_e_raw = np.zeros((len(temp[0]), 4))
    for i in range(len(temp[0])):
        quat_e_raw[i] = uq.RPY(temp[:,i], order = 'xyz').A
        if quat_e_raw[i, 2] < 0:
            quat_e_raw[i] = -quat_e_raw[i]
    #print(quat_e_raw)
    # four subplots one for each Qx Qy Qz
    fig, axs = plt.subplots(4, sharex=True, figsize=(12,8))
    # plot Qw
    axs[0].plot(time, quat_e_raw[:,0], 'r', label='Robot orientation')
    axs[0].plot(time, df_traj['Qw'], 'b', label='Trajectory orientation')
    axs[0].legend(["Robot", "Trajectory"])
    axs[0].set(ylabel='Qw')
    
    #axs[0].set_title('Qw')
    # plot Qx
    axs[1].plot(time, quat_e_raw[:,1], 'r', label='Robot orientation')
    axs[1].plot(time, df_traj['Qx'], 'b', label='Trajectory orientation')
    axs[1].set(ylabel='Qz')
    #axs[1].set_title('Qx')
    # plot Qy
    axs[2].plot(time, quat_e_raw[:,2], 'r', label='Robot orientation')
    axs[2].plot(time, df_traj['Qy'], 'b', label='Trajectory orientation')
    axs[2].set(ylabel='Qy')
    #axs[2].set_title('Qy')
    # plot Qz
    axs[3].plot(time, quat_e_raw[:,3], 'r', label='Robot orientation')
    axs[3].plot(time, df_traj['Qz'], 'b', label='Trajectory orientation')
    axs[3].set(ylabel='Qz')
    #axs[3].set_title('Qz')

    plt.xlabel("Time [s]")

    #grid on each subplot
    for ax in axs.flat:
        ax.grid()
    plt.tight_layout()
    plt.savefig('data/'+filename[:-4]+'_orientation.pdf')
    # plt.show()

def plot_position_error_individual(df_rob, df_traj):
    # plot the robot position
    fig, axs = plt.subplots(2, sharex=True, figsize=(12,4))
    axs[0].plot(time, df_rob['actual_position_0']-df_traj['x'], 'r', label='Robot position error x')
    axs[0].set(ylabel='x error [m]')
    axs[0].grid()
    axs[1].plot(time, df_rob['actual_position_1']-df_traj['y'], 'g', label='Robot position error y')
    axs[1].set(ylabel='y error [m]')
    axs[1].grid()

    plt.xlabel("Time [s]")

    # show the plot
    plt.savefig('data/'+filename[:-4]+'_position_error_individual.pdf')
    # plt.show()

def plot_position_error_individual_histogram(df_rob, df_traj):
    # plot the robot position
    fig, axs = plt.subplots(2, sharex=True, figsize=(12,6))
    axs[0].hist(df_rob['actual_position_0']-df_traj['x'], bins=100, color='r', label='Robot position error x')
    axs[0].set(xlabel='x error [m]')
    axs[1].hist(df_rob['actual_position_1']-df_traj['y'], bins=100, color='g', label='Robot position error y')
    axs[1].set(xlabel='y error [m]')
    # set labels
    for ax in axs.flat:
        ax.grid()
    #plt.ylabel("Frequency")
    fig.text(0.06, 0.5, 'Frequency', va='center', rotation='vertical')
    # show the plot
    plt.savefig('data/'+filename[:-4]+'_position_error_individual_histogram.pdf')
    # plt.show()

def plot_position_error_L2(df_rob, df_traj):
    # plot the robot position
    fig, axs = plt.subplots(1, figsize=(12,4))
    axs.plot(time, np.sqrt((df_rob['actual_position_0']-df_traj['x'])**2 + (df_rob['actual_position_1']-df_traj['y'])**2), 'r', label='Robot position error L2')
    # set labels
    axs.set(ylabel='L2 Error [m]')
    axs.grid()
    plt.xlabel("Time [s]")
    # show the plot
    plt.savefig('data/'+filename[:-4]+'_position_error_L2.pdf')
    # plt.show()

def plot_position_error_L2_histogram(df_rob, df_traj):
    # plot the robot position
    fig, axs = plt.subplots(1, figsize=(12,4))
    axs.hist(np.sqrt((df_rob['actual_position_0']-df_traj['x'])**2 + (df_rob['actual_position_1']-df_traj['y'])**2), bins=100, color='r', label='Robot position error L2')
    # set labels
    axs.set(xlabel='L2 Error [m]', ylabel='Frequency')
    axs.grid()
    # show the plot
    plt.savefig('data/'+filename[:-4]+'_position_error_L2_histogram.pdf')
    # plt.show()

def plot_orientation_error_individual(df_rob, df_traj):
    # turn orientation into unit quaternions
    # make numpy array of the robot orientation in rpy xyz 
    temp = np.array([df_rob['actual_position_3'], df_rob['actual_position_4'], df_rob['actual_position_5']])
    # make numpy array of uq.RPY with length of temp
    quat_e_raw = np.zeros((len(temp[0]), 4))
    for i in range(len(temp[0])):
        quat_e_raw[i] = uq.RPY(temp[:,i], order = 'xyz').A
        if quat_e_raw[i, 2] < 0:
            quat_e_raw[i] = -quat_e_raw[i]
    #print(quat_e_raw)

    # plot the orientation error
    fig, axs = plt.subplots(4, sharex=True, figsize=(12,8))
    # plot Qw
    axs[0].plot(time, quat_e_raw[:,0]-df_traj['Qw'], 'r', label='Robot orientation error Qw')
    axs[0].set(ylabel='Qw error')
    axs[0].grid()
    # plot Qx
    axs[1].plot(time, quat_e_raw[:,1]-df_traj['Qx'], 'r', label='Robot orientation error Qx')
    axs[1].set(ylabel='Qx error')
    axs[1].grid()
    # plot Qy
    axs[2].plot(time, quat_e_raw[:,2]-df_traj['Qy'], 'r', label='Robot orientation error Qy')
    axs[2].set(ylabel='Qy error')
    axs[2].grid()
    # plot Qz
    axs[3].plot(time, quat_e_raw[:,3]-df_traj['Qz'], 'r', label='Robot orientation error Qz')
    axs[3].set(ylabel='Qz error')
    axs[3].grid()

    plt.xlabel("Time [s]")
    # show the plot
    plt.savefig('data/'+filename[:-4]+'_orientation_error_individual.pdf')
    # plt.show()

def plot_orientation_error_individual_histogram(df_rob, df_traj):
    # turn orientation into unit quaternions
    # make numpy array of the robot orientation in rpy xyz 
    temp = np.array([df_rob['actual_position_3'], df_rob['actual_position_4'], df_rob['actual_position_5']])
    # make numpy array of uq.RPY with length of temp
    quat_e_raw = np.zeros((len(temp[0]), 4))
    for i in range(len(temp[0])):
        quat_e_raw[i] = uq.RPY(temp[:,i], order = 'xyz').A
        if quat_e_raw[i, 2] < 0:
            quat_e_raw[i] = -quat_e_raw[i]
    #print(quat_e_raw)
    # plot the orientation error
    fig, axs = plt.subplots(4, figsize=(12,8))
    # plot Qw
    axs[0].hist(quat_e_raw[:,0]-df_traj['Qw'], bins=100, color='r', label='Robot orientation error Qw')
    axs[0].set(xlabel='Qw error')
    axs[0].grid()
    # plot Qx
    axs[1].hist(quat_e_raw[:,1]-df_traj['Qx'], bins=100, color='r', label='Robot orientation error Qx')
    axs[1].set(xlabel='Qx error')
    axs[1].grid()
    # plot Qy
    axs[2].hist(quat_e_raw[:,2]-df_traj['Qy'], bins=100, color='r', label='Robot orientation error Qy')
    axs[2].set(xlabel='Qy error')
    axs[2].grid()
    # plot Qz
    axs[3].hist(quat_e_raw[:,3]-df_traj['Qz'], bins=100, color='r', label='Robot orientation error Qz')
    axs[3].set(xlabel='Qz error')
    axs[3].grid()

    plt.subplots_adjust(hspace=0.5)

    fig.text(0.06, 0.5, 'Frequency', va='center', rotation='vertical')

    # show the plot
    plt.savefig('data/'+filename[:-4]+'_orientation_error_individual_histogram.pdf')
    # plt.show()

######################
# get wanted velocity and acceleration data from trajectory position
######################
# combine df_traj x and y into one np array
timestep = 0.002 # 2ms
pos_xy = np.array([df_traj['x'], df_traj['y']])
vel_xy = np.gradient(pos_xy, axis=1) / timestep
acc_xy = np.gradient(vel_xy, axis=1) / timestep

# compare vel_xy with df_rob control_velocity_0 and control_velocity_1
def plot_velocity(df_rob, vel_xy):
    # plot the robot velocity
    fig, axs = plt.subplots(2, sharex=True, figsize=(12,6))
    axs[0].plot(time, df_rob['control_velocity_0'], 'r', label='Robot velocity x')
    axs[0].plot(time, vel_xy[0], 'b', label='Trajectory velocity x')
    axs[0].set(ylabel='x velocity [m/s]')
    axs[0].grid()
    axs[1].plot(time, df_rob['control_velocity_1'], 'r', label='Robot velocity y')
    axs[1].plot(time, vel_xy[1], 'b', label='Trajectory velocity y')
    axs[1].set(ylabel='y velocity [m/s]')
    axs[1].grid()

    # set legend
    axs[0].legend(["Robot", "Trajectory"])

    plt.xlabel("Time [s]")

    plt.savefig('data/'+filename[:-4]+'_velocity.pdf')
    # plt.show()

def plot_velocity_error(df_rob, vel_xy):
    # plot the robot velocity
    fig, axs = plt.subplots(2, sharex=True, figsize=(12,6))
    axs[0].plot(time, df_rob['control_velocity_0']-vel_xy[0], 'r', label='Robot velocity error x')
    axs[0].set(ylabel='x velocity error [m/s]')
    axs[0].grid()
    axs[1].plot(time, df_rob['control_velocity_1']-vel_xy[1], 'g', label='Robot velocity error y')
    axs[1].set(ylabel='y velocity error [m/s]')
    axs[1].grid()

    plt.xlabel("Time [s]")
    # show the plot
    plt.savefig('data/'+filename[:-4]+'_velocity_error.pdf')
    # plt.show()

def plot_velocity_error_histogram(df_rob, vel_xy):
    # plot the robot velocity
    fig, axs = plt.subplots(2, sharex=True, figsize=(12,6))
    axs[0].hist(df_rob['control_velocity_0']-vel_xy[0], bins=100, color='r', label='Robot velocity error x')
    axs[0].set(xlabel='x velocity error [m/s]')
    axs[0].grid()
    axs[1].hist(df_rob['control_velocity_1']-vel_xy[1], bins=100, color='g', label='Robot velocity error y')
    axs[1].set(xlabel='y velocity error [m/s]')
    axs[1].grid()

    fig.text(0.06, 0.5, 'Frequency', va='center', rotation='vertical')
    # show the plot
    plt.savefig('data/'+filename[:-4]+'_velocity_error_histogram.pdf')
    # plt.show()

def plot_accelerations(df_rob, acc_xy):
    # plot the robot velocity
    fig, axs = plt.subplots(2, sharex=True, figsize=(12,6))
    axs[0].plot(time, df_rob['control_acceleration_0'], 'r', label='Robot acceleration x')
    axs[0].plot(time, acc_xy[0], 'b', label='Trajectory acceleration x')
    axs[0].set(ylabel='x acceleration [m/s^2]')
    axs[0].grid()
    axs[1].plot(time, df_rob['control_acceleration_1'], 'r', label='Robot acceleration y')
    axs[1].plot(time, acc_xy[1], 'b', label='Trajectory acceleration y')
    axs[1].set(ylabel='y acceleration [m/s^2]')
    axs[1].grid()

    # set legend
    axs[0].legend(["Robot", "Trajectory"])

    plt.xlabel("Time [s]")

    plt.savefig('data/'+filename[:-4]+'_acceleration.pdf')
    # plt.show()

def plot_accelerations_error(df_rob, acc_xy):
    # plot the robot velocity
    fig, axs = plt.subplots(2, sharex=True, figsize=(12,6))
    axs[0].plot(time, df_rob['control_acceleration_0']-acc_xy[0], 'r', label='Robot acceleration error x')
    axs[0].set(ylabel='x acceleration error [m/s^2]')
    axs[0].grid()
    axs[1].plot(time, df_rob['control_acceleration_1']-acc_xy[1], 'g', label='Robot acceleration error y')
    axs[1].set(ylabel='y acceleration error [m/s^2]')
    axs[1].grid()
    
    plt.xlabel("Time [s]")

    # show the plot
    plt.savefig('data/'+filename[:-4]+'_acceleration_error.pdf')
    # plt.show()

def plot_accelerations_error_histogram(df_rob, acc_xy):
    # plot the robot velocity
    fig, axs = plt.subplots(2, sharex=True, figsize=(12,6))
    axs[0].hist(df_rob['control_acceleration_0']-acc_xy[0], bins=100, color='r', label='Robot acceleration error x')
    axs[0].set(xlabel='x acceleration error [m/s^2]')
    axs[0].grid()
    axs[1].hist(df_rob['control_acceleration_1']-acc_xy[1], bins=100, color='g', label='Robot acceleration error y')
    axs[1].set(xlabel='y acceleration error [m/s^2]')
    axs[1].grid()
    #set limits
    axs[0].set_xlim(-0.2, 0.2)

    fig.text(0.06, 0.5, 'Frequency', va='center', rotation='vertical')

    # show the plot
    plt.savefig('data/'+filename[:-4]+'_acceleration_error_histogram.pdf')
    # plt.show()

def plot_force(df_rob):
    # plot the robot force
    fig, axs = plt.subplots(1, sharex=True, figsize=(12,4))
    axs.plot(time, df_rob['actual_force_2'], 'r', label='Robot force z')
    axs.plot(time, df_rob['actual_force_filtered_2'], 'g', label='Robot force filtered z')
    axs.plot(time, df_rob['force_reference_2'], 'b', label='Trajectory force z')
    
    axs.set(ylabel='z force [N]')
    axs.grid()
    

    # set legend
    axs.legend(["Robot Measurement", "Filtered Measurement", "Reference"])

    plt.xlabel("Time [s]")
        
    plt.savefig('data/'+filename[:-4]+'_force.pdf')
    # plt.show()

def plot_force_error(df_rob):
    # plot the robot force
    fig, axs = plt.subplots(2, sharex=True, figsize=(12,6))
    axs[0].plot(time, df_rob['actual_force_2']-df_rob['force_reference_2'], 'b', label='Robot force error z')
    axs[0].set(ylabel='Measured z force error [N]')
    axs[0].grid()
    axs[1].plot(time, df_rob['actual_force_filtered_2']-df_rob['force_reference_2'], 'b', label='Robot force filtered error z')
    axs[1].set(ylabel='Filtered z force error [N]')
    axs[1].grid()

    plt.xlabel("Time [s]")
    # show the plot
    plt.savefig('data/'+filename[:-4]+'_force_error.pdf')
    # plt.show()

def plot_force_error_histogram(df_rob):
    # plot the robot force
    fig, axs = plt.subplots(2, sharex=True, figsize=(12,6))
    axs[0].hist(df_rob['actual_force_2']-df_rob['force_reference_2'], bins=100, color='b', label='Robot force error z')
    axs[0].set(xlabel='z force error [N]')
    axs[0].grid()
    axs[1].hist(df_rob['actual_force_filtered_2']-df_rob['force_reference_2'], bins=100, color='b', label='Robot force filtered error z')
    axs[1].set(xlabel='z force error [N]')
    axs[1].grid()

    fig.text(0.06, 0.5, 'Frequency', va='center', rotation='vertical')
    # show the plot
    plt.savefig('data/'+filename[:-4]+'_force_error_histogram.pdf')
    # plt.show()

def plot_3d_position_color_force(df_rob):
    # plot the robot position
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    sc = ax.scatter(df_rob['actual_position_0'], df_rob['actual_position_1'], df_rob['actual_position_2'], c=df_rob['actual_force_filtered_2'], cmap='jet', vmin=0, vmax=15)
    # set labels
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.zaxis.set_rotate_label(False)
    ax.set_zlabel('Z [m]', rotation=90, labelpad=10)
    ax.view_init(elev=30, azim=-235)
    ax.set_zlim(-0.008,-0.004)
    
    # set title
    ax.set_title('Contact Forces')
    cbar = fig.colorbar(sc, ax=ax, pad=0.05, aspect=40, location='left')  # Move color bar to the right
    cbar.set_label('Force Magnitude [N]')  
    # show the plot
    plt.savefig('data/'+filename[:-4]+'_3d_position_color_force.pdf')
    # plt.show()

# call the function
plot_position(df_rob, df_traj) 
plot_orientation(df_rob, df_traj)
plot_position_error_individual(df_rob, df_traj)
plot_position_error_individual_histogram(df_rob, df_traj)
plot_position_error_L2(df_rob, df_traj)
plot_position_error_L2_histogram(df_rob, df_traj)
plot_orientation_error_individual(df_rob, df_traj)
plot_orientation_error_individual_histogram(df_rob, df_traj)
plot_velocity(df_rob, vel_xy)
plot_velocity_error(df_rob, vel_xy)
plot_velocity_error_histogram(df_rob, vel_xy)
plot_accelerations(df_rob, acc_xy)
plot_accelerations_error(df_rob, acc_xy)
plot_accelerations_error_histogram(df_rob, acc_xy)
plot_force(df_rob)
plot_force_error(df_rob)
plot_force_error_histogram(df_rob)
plot_3d_position_color_force(df_rob)
