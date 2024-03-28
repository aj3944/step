import numpy as np
import time
import accelerometer 


device=accelerometer.Accelerometer()

x_com_prev = 0
# y_com_prev = a
z_com_prev = 0.27

x_zmp_prev = 0.05
y_zmp_prev = 0
z_zmp_prev = 0.035

dt = 0.1

x_dot_com_prev = 0 
u_prev = 0

x_des_com = 0
x_des_dot_com = 0
x_des_zmp = 0
Kp = 20
Kd = 0.4
K  = 10
m = 1.3
error = 0
while True:
    ax,ay,az = device.readAccData()
    theta = np.arctan(ax/az)
    
    # print("Acceleration in Z", az)    
    
    if theta <= 0.06:
        theta = 0 


    print("orientation of the robot", theta) 
    F = m *  np.linalg.norm([ax,ay, az])   
    print("Force" , F)
    #Can replace az with m*
    # x_dot_com = x_dot_com_prev +((dt* az * x_com_prev ) / z_com_prev ) - ((dt * az * x_zmp_prev) / z_com_prev)

    x_dot_com = x_dot_com_prev +((dt *  F * np.sin(theta) * x_com_prev) / (m * z_com_prev )) - ((dt * F * np.sin(theta) * x_zmp_prev) / (m * z_com_prev))

    print("Velocity of COM" , x_dot_com)
    x_com = x_com_prev + (x_dot_com_prev * dt)
    
    # x_zmp = x_zmp_prev + (u_prev * dt )
    
    #PD control to achieve desired acceleration
    
    x_dot_dot_com = Kp*(x_des_com - x_com) + Kd*(x_des_dot_com - x_dot_com)

    

    if (-1 <= x_dot_dot_com <= 1):
        x_dot_dot_com = 0 



    print("Acceleratiom of COM", x_dot_dot_com)

    error = error + (x_dot_dot_com * dt)

    u = K * (0 - x_dot_dot_com) 

    if u < 0 : 
        u = np.absolute(u)

    if u >= 240:
        u = 240

    # u = K * (x_des_zmp - x_zmp)

    print(u)
    

    # x_dot_zmp = (x_zmp - x_zmp_prev)/dt
    # ang_ctrl =    
    # u_prev = u
    
    # #Optional to control angles of motor
    # ang_ctrl_prev = np.arccos((x_com - x_zmp)/z_com_prev)
    
    # u = K *( 0 - ang_ctrl_prev)
    
    x_dot_com_prev, _ , _ = device.readGyroData()
    x_com_prev = x_com + (x_dot_com_prev*dt)
    # x_zmp_prev = x_zmp

        


    
    
    
    
    
