

import numpy as np
from tqdm import tqdm
import spatialmath as sm
from spatialmath.base import q2r, r2x, rotx, roty, rotz, r2q, q2r, v2q
import roboticstoolbox as rtb
import matplotlib.pyplot as plt

class Trajectory():
    def __init__(self, filename):
        with open(filename, 'r') as f:
            data = f.readlines()

        self.traj = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]*len(data)).reshape(len(data), 7)
        self.time = np.zeros(len(data))
        dt = 1/500.0

        for i, line in enumerate(data):
            line = line.split(', ')
            #remove the last element which is a newline character
            line[-1] = line[-1].strip()
            self.time[i] = float(i*dt)
            for j in range(7):
                self.traj[i][j] = float(line[j])
        self.dtraj = np.gradient(self.traj, axis=0) / dt
        self.ddtraj = np.gradient(self.dtraj, axis=0) / dt

        self.p = self.traj[:,[0,1,4,5,6]]
        self.dp = np.gradient(self.p, axis=0) / dt
        self.ddp = np.gradient(self.dp, axis=0) / dt
        
    def plot(self):
        plt.figure()
        #make 3x3 subplots with traj, dtraj and ddtraj 0 to 3
        for i in range(3):
            plt.subplot(3, 3, i+1)
            plt.plot(self.time, self.traj[:,i])
            plt.title(f"Traj {i}")
        for i in range(3):
            plt.subplot(3, 3, i+4)
            plt.plot(self.time, self.dtraj[:,i])
            plt.title(f"dTraj {i}")
        for i in range(3):
            plt.subplot(3, 3, i+7)
            plt.plot(self.time, self.ddtraj[:,i])
            plt.title(f"ddTraj {i}")
        plt.show()


if __name__ == "__main__":
    traj = Trajectory("shorttrajectory.txt")
    traj.plot()