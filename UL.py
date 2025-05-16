import rtde_control

#put correct IP adress of the robot and make sure it is in remote control mode
rtde_c = rtde_control.RTDEControlInterface("10.54.6.36")

#loop poses 10 times
for _ in range(10):    
    #add the correct joint angles (in radians) for the two robot poses    
    rtde_c.moveJ([0.0, -3.3161, -0.1745, -1.5708, 0.0, -0.1745], 3.14, 3.14)    
    rtde_c.moveJ([0.0, -2.9671, 0.1745, -1.5708, 0.0, 0.1745], 3.14, 3.14)

# 14.18 - 110mm
# 14.43 - 65.5mm