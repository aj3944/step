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



g = -9.81            # Acceleration due to gravity (m/s^2)
L = 0.290            # Length of pendulum (m)
m = 6                # Mass of pendulum (kg)
b = 0.5              # Damping coefficient
Kp = 100            # Proportional gain
Kd = 0.01             # Derivative gain
#Ki = 0.1            # Integral gain


# Initial conditions
theta0 = 17 * np.pi / 18      # Initial angle (radians)
theta_dot0 = 0                # Initial angular velocity (radians/s)

# Simulation parameters
dt = 0.1                        # Time step (s)
t = np.arange(0,10,0.1)
theta = [0]
theta_dot = [0]
u = [0]

# theta_dot_imu_x = tuple(device.readGyroData())[0]
# theta_dot_imu_x = theta_dot_imu[0]
theta_dot_GCOM =[0]

theta[0] = theta0
theta_dot[0] = theta_dot0
theta_dot_des = 0
theta_des = np.pi



# theta_dot_imu[0] = 0
i = 0;


control_angle = 20;


IMU = Transformation()
COM = Transformation()
GCOM = Transformation()
ANKLE = Transformation()
HIP = Transformation()
WORLD = Transformation()


COM.tanslation = [0.04285,0,0.0062]
GCOM.tanslation = [0,0,-0.290]
ANKLE.translation = [0,0, 0.045]

HIP.rotation = R.from_rotvec([0., control_angle, 0., ])
HIP.translation = [0,0,.290]


IMU_TO_HIP = HIP * ANKLE * GCOM * COM * IMU;
# IMU_TO_HIP = HIP  * IMU;

print(IMU_TO_HIP.T_matrix());





if is_jetson:

    device = accelerometer.Accelerometer()


    mark_4 = Bot()
    mark_4.home()
    mark_4.read()
    time.sleep(1)


# BODY = Transformation()
# COM = Transformation()
# COM = Transformation()



while is_jetson:
    i += 1 
    theta_dot_imu_x = tuple(device.readGyroData())[0]





    ang_ctrl_GCOM = Kp * theta_dot_imu_x ;

    print("Angle values in GCOM frame",ang_ctrl_GCOM)
    '''
    Ang_ctrl_GCOM to hip
    '''

    deg_hip = np.rad2deg(ang_ctrl_GCOM)


    mark_4.injest_ik([0,0,0,0,-deg_hip,-deg_hip])

    '''
    Motor values for the hip
    '''
    print(i,deg_hip)
