import csv  
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os

#go through each file in the folder and plot the force and position data and save the plots in a subfolder with the same name as the file
for file in os.listdir("data"):
    if file.endswith(".csv"):
        df = pd.read_csv("data/" + file)

        #filename without extension
        filename = file.split(".")[0]

        #mkdir if not exists with name filename
        if not os.path.exists("data/" + filename):
            os.makedirs("data/" + filename)


        # extract actual_force_2 and desired_force_2
        actual_force = df["actual_force_2"]
        filtered_force = df["actual_force_filtered_2"]
        desired_force = df["force_reference_2"]
        time = np.arange(0, len(actual_force)*0.002, 0.002)

        # Plot the data
        plt.figure()
        plt.plot(time, actual_force, label='Actual Force')
        plt.plot(time, filtered_force, label='Filtered Force')
        plt.plot(time, desired_force, label='Desired Force')
        plt.xlabel('Time (s)')
        plt.ylabel('Force (N)')
        plt.title('Force along z-axis')
        plt.legend(["Actual Force", "Filtered Force", "Desired Force"])
        plt.savefig('data/' + filename + '/force_plot_long.pdf')


        # plot the first 5 seconds
        cutoff = 5
        dt = 0.002
        last_index = int(cutoff/dt)
        plt.figure()
        plt.plot(time[:last_index], actual_force[:last_index], label='Actual Force')
        plt.plot(time[:last_index], filtered_force[:last_index], label='Filtered Force')
        plt.plot(time[:last_index], desired_force[:last_index], label='Desired Force')
        plt.xlabel('Time (s)')
        plt.ylabel('Force (N)')
        plt.title('Force along z-axis')
        plt.legend(["Actual Force", "Filtered Force", "Desired Force"])
        plt.savefig('data/' + filename + '/force_plot.pdf')


        # position in 7 subfigures
        actual_position_0 = df["positions_as_quat_0"]
        desired_position_0 = df["target_positions_as_quat_0"]
        actual_position_1 = df["positions_as_quat_1"]
        desired_position_1 = df["target_positions_as_quat_1"]
        actual_position_2 = df["positions_as_quat_2"]
        desired_position_2 = df["target_positions_as_quat_2"]
        actual_position_3 = df["positions_as_quat_3"]
        desired_position_3 = df["target_positions_as_quat_3"]
        actual_position_4 = df["positions_as_quat_4"]
        desired_position_4 = df["target_positions_as_quat_4"]
        actual_position_5 = df["positions_as_quat_5"]
        desired_position_5 = df["target_positions_as_quat_5"]
        actual_position_6 = df["positions_as_quat_6"]
        desired_position_6 = df["target_positions_as_quat_6"]


        plt.figure()
        fig, axs = plt.subplots(7, 1, figsize=(20, 16))
        fig.suptitle('Position along x-axis')
        axs[0].plot(time, actual_position_0, label='Actual Position')
        axs[0].plot(time, desired_position_0, label='Desired Position')
        axs[0].legend(["Actual Position", "Desired Position"])
        axs[1].plot(time, actual_position_1, label='Actual Position')
        axs[1].plot(time, desired_position_1, label='Desired Position')
        axs[2].plot(time, actual_position_2, label='Actual Position')
        axs[2].plot(time, desired_position_2, label='Desired Position')
        axs[3].plot(time, actual_position_3, label='Actual Position')
        axs[3].plot(time, desired_position_3, label='Desired Position')
        axs[4].plot(time, actual_position_4, label='Actual Position')
        axs[4].plot(time, desired_position_4, label='Desired Position')
        axs[5].plot(time, actual_position_5, label='Actual Position')
        axs[5].plot(time, desired_position_5, label='Desired Position')
        axs[6].plot(time, actual_position_6, label='Actual Position')
        axs[6].plot(time, desired_position_6, label='Desired Position')

        plt.savefig('data/' + filename + '/position_plot_long.pdf')

        plt.figure()
        fig, axs = plt.subplots(7, 1, figsize=(20, 16))
        fig.suptitle('Position along x-axis')
        axs[0].plot(time[:last_index], actual_position_0[:last_index], label='Actual Position')
        axs[0].plot(time[:last_index], desired_position_0[:last_index], label='Desired Position')
        axs[0].legend(["Actual Position", "Desired Position"])
        axs[1].plot(time[:last_index], actual_position_1[:last_index], label='Actual Position')
        axs[1].plot(time[:last_index], desired_position_1[:last_index], label='Desired Position')
        axs[2].plot(time[:last_index], actual_position_2[:last_index], label='Actual Position')
        axs[2].plot(time[:last_index], desired_position_2[:last_index], label='Desired Position')
        axs[3].plot(time[:last_index], actual_position_3[:last_index], label='Actual Position')
        axs[3].plot(time[:last_index], desired_position_3[:last_index], label='Desired Position')
        axs[4].plot(time[:last_index], actual_position_4[:last_index], label='Actual Position')
        axs[4].plot(time[:last_index], desired_position_4[:last_index], label='Desired Position')
        axs[5].plot(time[:last_index], actual_position_5[:last_index], label='Actual Position')
        axs[5].plot(time[:last_index], desired_position_5[:last_index], label='Desired Position')
        axs[6].plot(time[:last_index], actual_position_6[:last_index], label='Actual Position')
        axs[6].plot(time[:last_index], desired_position_6[:last_index], label='Desired Position')

        plt.savefig('data/' + filename + '/position_plot.pdf')










