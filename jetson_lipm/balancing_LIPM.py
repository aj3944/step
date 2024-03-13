import socket
is_jetson = socket.gethostname() == 'nano'
 

import numpy as np
import time

if is_jetson:
    import accelerometer 
    from bot import Bot

from quaternion import Quaternion as qt
from quaternion import Haal,Transformation

from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt



g = -9.81            # Acceleration due to gravity (m/s^2)
L = 0.290            # Length of pendulum (m)
m = 2               # Mass of pendulum (kg)
b = 0             # Damping coefficient
Kp = 45  # Proportional gain
Ki = 0.1          # Integral gain


# Initial conditions
# theta_COM_init = 0.1745     #10 degrees # Initial angle (radians)
# theta_dot_COM_init = 0                # Initial angular velocity (radians/s)


theta_imu_prev = 0
theta_applied_prev = 0.175
theta_dot_applied_prev = 0

integral = 0

u = []
u.append(0)

# Simulation parameters
dt = 0.1            # Time step (s)




i = 0


# IMU = Transformation()
# COM = Transformation()
# GCOM = Transformation()
# ANKLE = Transformation()
# HIP = Transformation
# WORLD = Transformation()



from math import atan2

if is_jetson:

    device = accelerometer.Accelerometer()


    mark_4 = Bot()
    mark_4.home()
    mark_4.read()
    time.sleep(1)

while i < 1000:
    ax,ay,az = device.readAccData()
    gx,gy,gz = device.readGyroData()

    pitch = np.rad2deg(atan2(ax,az));
    roll = np.rad2deg(atan2(ay,az));
    print(ax,ay,az,pitch,roll);
    roll_rad = atan2(ay,az);

    # time.sleep(0.01)
# while is_jetson: 
    # i += 1 
    theta_dot_imu = tuple(device.readGyroData())

    # print("theta_dot_imu" , theta_dot_imu)

    theta_dot_imu = np.linalg.norm(theta_dot_imu)

    if i %200 == 0:
        theta_imu_prev = roll_rad;

    theta_imu = theta_imu_prev + (dt* theta_dot_imu)
    # print("theta_imu", theta_imu)

    theta_imu_prev = theta_imu

    '''
    Calculated Angular accelereation of the COM
    '''

    # theta_dot_dot_applied = ((g / L ) * np.sin(theta_imu)) + ((b * theta_dot_imu) / (m * (L ** 2)) ) + (u[i] / (m  * (L**2) ) )   
    theta_dot_dot_applied = ((g / L ) * np.sin(theta_imu)) + ((b * theta_dot_imu) / (m * (L ** 2)) ) + (u[i] / (m  * (L**2) ) )   
    # print("theta_dot_dot_applied",theta_dot_dot_applied)

    theta_dot_applied = theta_dot_applied_prev + (dt * theta_dot_dot_applied)
    # print("theta_dot_applied",theta_dot_applied)
 
    theta_applied = theta_applied_prev + (dt * theta_dot_applied)
    theta_applied_prev = theta_applied

    theta_dot_applied_prev = theta_dot_applied


    # print("theta_applied", theta_applied)

    motor_angle = np.rad2deg(theta_applied) 

    # print("motor_angle" , motor_angle)

    plt.scatter(i, motor_angle)

    # mark_4.injest_ik([20,-10,0,0,motor_angle,0])
    # mark_4.injest_ik([20,-10,0,0,motor_angle,0])
    # mark_4.injest_ik([0,0,25,-50,motor_angle,0])
    # mark_4.injest_ik([0,0,25,-50,motor_angle,0])


    # tau =(m * (L**2) * (theta_dot_dot_applied))

    # u.append(tau)

    # integral += tau * dt

    adj_tau = Kp * (0- theta_applied)  #Ki * (integral) 
    # print("adj_tau",adj_tau)
    # print("-----------------")
    u.append(adj_tau)

    i+=1
    time.sleep(0.01)

 
    # if i==10:
    #      break


plt.show()
# plt.draw()
# plt.pause(0.05)
                


