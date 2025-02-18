import numpy as np
from tqdm import tqdm
import spatialmath as sm
from spatialmath.base import q2r, r2x, rotx, roty, rotz, r2q, q2r, v2q
import roboticstoolbox as rtb
import matplotlib.pyplot as plt

poi = np.array([
    [93, 243],
    [130, 243],
    [130, 223],
    [175, 223],
    [175, 263],
    [130, 263],
    [130, 223],
    [175, 223],
    [175, 243],
    [213, 243],
    [213, 223],
    [254, 223],
    [254, 164],
    [334, 164],
    [334, 144],
    [379, 144],
    [379, 184],
    [334, 184],
    [334, 144],
    [379, 144],
    [379, 164],
    [663, 164],
    [663, 271],
    [648, 271],
    [648, 289],
    [663, 289],
    [663, 392],
    [379, 392],
    [379, 372],
    [334, 372],
    [334, 412],
    [379, 412],
    [379, 372],
    [334, 372],
    [334, 392],
    [254, 392],
    [254, 339],
    [213, 339],
    [213, 319],
    [175, 319],
    [175, 339],
    [130, 339],
    [130, 319],
    [93, 319],
    [130, 319],
    [130, 299],
    [175, 299],
    [175, 319],
    [213, 319],
    [213, 299],
    [297, 299],
    [297, 339],
    [213, 339],
    [213, 299],
    [297, 299],
    [297, 319],
    [334, 319],
    [334, 299],
    [379, 299],
    [379, 339],
    [334, 339],
    [334, 299],
    [379, 299],
    [379, 319],
    [435, 319],
    [435, 292],
    [444, 281],
    [480, 281],
    [480, 261],
    [525, 261],
    [525, 281],
    [565, 281],
    [565, 301],
    [648, 301],
    [648, 261],
    [565, 261],
    [565, 281],
    [525, 281],
    [525, 301],
    [480, 301],
    [480, 281],
    [444, 281],
    [435, 272],
    [425, 281],
    [435, 292],
    [444, 281],
    [435, 272],
    [435, 243],
    [379, 243],
    [379, 263],
    [334, 263],
    [334, 243],
    [297, 243],
    [297, 263],
    [213, 263],
    [213, 223],
    [297, 223],
    [297, 243],
    [334, 243],
    [334, 223],
    [379, 223],
    [379, 243]
])

poi = poi + np.array([0, -281])

print(poi)
#plt.plot(poi[:, 0], poi[:, 1])
#plt.show()



# Generate trajectory

#startpos = np.array([0, 0, 0])
T0 = sm.SE3.Trans(0.0, 0, 0.301) * sm.SE3.RPY([0, np.pi, 0]) # sm.SE3.Trans(0.5, 0, 0.3) * sm.SE3.RPY([0, np.pi, 0]) changed z from 0.3 to 0.4 to avoid collision
T1 = sm.SE3.Trans(0.0, 0, 0.301) * sm.SE3.RPY([0, np.pi, 0])

dt = 0.002 
endTime = 0.3
n = int(endTime/dt)

ctr = np.array([])
times = np.array([])
startTime = 0
scalingX = 5000 / 2
scalingY = 2500 / 2
movementX = -0.66
movementY = 0.2



#for i in tqdm(range(len(poi)-1)):
for i in tqdm(range(0, 5)):
    distance = np.sqrt((poi[i+1, 0] - poi[i, 0])**2 + (poi[i+1, 1] - poi[i, 1])**2)
    endTime = distance*0.05
    n = int(endTime/dt)
    T0.t[0] = movementX + poi[i, 0]/scalingX
    T0.t[1] = movementY + poi[i, 1]/scalingY
    T1.t[0] = movementX + poi[i+1, 0]/scalingX
    T1.t[1] = movementY + poi[i+1, 1]/scalingY
    trajectory = rtb.ctraj(T0, T1, n)
    time = np.linspace(startTime, startTime+endTime, n)
    startTime = startTime+endTime
    if i == 0:
        ctr = trajectory
        times = time
    else:
        x = ctr.Alloc(len(ctr)+len(trajectory))
        for j in range(0, len(ctr)):
            x[j] = ctr[j]
        for j in range(0, len(trajectory)):
            x[j+len(ctr)] = trajectory[j]
        ctr = x
        times = np.append(times, time)



        
print(f"No of points in trajectory: {len(ctr)}") 

#open file
with open("shorttrajectory.txt", "w") as file:
    for T in ctr:
        rotation = T.R
        quaternion = r2q(rotation)
        file.write(f"{T.t[0]:.15f}, {T.t[1]:.15f}, {T.t[2]:.15f}, {quaternion[0]:.15f}, {quaternion[1]:.15f}, {quaternion[2]:.15f}, {quaternion[3]:.15f}\n")
