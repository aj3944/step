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
Kp = 100             # Proportional gain
Ki = 0.01            # Integral gain


# Initial conditions
# theta_COM_init = 0.1745     #10 degrees # Initial angle (radians)
# theta_dot_COM_init = 0                # Initial angular velocity (radians/s)


theta_COM_prev = 0.1745
theta_dot_COM_prev = 0

theta_ANKLE = theta_COM_prev


integral = 0

u = []
u.append(0)

# Simulation parameters
dt = 0.1                        # Time step (s)
t = np.arange(0,10,0.1)

theta_dot_des = 0
theta_des = 0



# theta_dot_imu[0] = 0
i = 0


control_angle = 20


IMU = Transformation()
COM = Transformation()
GCOM = Transformation()
ANKLE = Transformation()
HIP = Transformation()
WORLD = Transformation()


COM.translation = [0.04285,0,0.0062]
GCOM.translation = [0,0,-L]
ANKLE.translation = [0,0, 0.045]

HIP.rotation = R.from_rotvec([0., control_angle, 0., ])
HIP.translation = [0,0,L]


IMU_TO_HIP = HIP * ANKLE * GCOM * COM * IMU
IMU_TO_COM = COM * IMU

IMU_TO_GCOM =  GCOM * COM * IMU
IMU_TO_ANKLE = ANKLE * IMU_TO_GCOM

# print(IMU_TO_HIP.T_matrix())
print(IMU_TO_COM.translation)

def cross_product(vector1, vector2):
    """
    Compute the cross product of two 3D vectors.

    Parameters:
    - vector1: A list or tuple representing the first vector [x1, y1, z1].
    - vector2: A list or tuple representing the second vector [x2, y2, z2].

    Returns:
    - The cross product vector as a list [x, y, z].
    """
    x1, y1, z1 = vector1
    x2, y2, z2 = vector2

    x = y1 * z2 - y2 * z1
    y = -(x1 * z2 - x2 * z1)
    z = x1 * y2 - x2 * y1

    return [x, y, z]

def angular_vel_transform(angular_vel , translation_vector):

    theta_dot_transform = angular_vel + cross_product(translation_vector , angular_vel)
    return theta_dot_transform




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
    # i += 1 
    theta_dot_imu_x = tuple(device.readGyroData())[0]
    
    theta_dot_IMU_TO_COM = angular_vel_transform([theta_dot_imu_x , 0 , 0] , IMU_TO_COM.translation)

    theta_dot_IMU_TO_ANKLE = angular_vel_transform(theta_dot_IMU_TO_COM , IMU_TO_ANKLE.translation )


    '''
    Calculated Angular accelereation of the COM
    '''

    theta_dot_dot_COM = g / L * np.sin(theta_COM_prev) + b / (m * L ** 2) * theta_dot_COM_prev + u[i] / (m * L ** 2)

    theta_dot_COM = theta_dot_COM_prev + dt * theta_dot_dot_COM
    theta_dot_COM_prev = theta_dot_COM

    thetaCOM = theta_COM_prev + dt * theta_dot_COM
    theta_COM_prev = thetaCOM

    tau = m * L ** 2 * (theta_dot_COM - theta_dot_IMU_TO_ANKLE[i]) / dt

    u.append(tau)

    '''
    PI control for Ankle
    '''
    theta_error = theta_des - theta_ANKLE_prev

    integral += theta_error * dt

    ang_ctrl_ANKLE = Kp* (theta_error) + Ki * (integral)

    # ang_ctrl_ankle= Kp * (theta_des - theta[i - 1]) + Kd * (theta_dot_des - theta_dot[i - 1])

    hip_motor = np.rad2deg(ang_ctrl_ANKLE)

    '''
    Integrating theta_dot_IMU_TO_ANKLE
    '''

    theta_ANkle= theta_ANKLE_prev+ dt * theta_dot_IMU_TO_ANKLE
    
    theta_ANKLE_prev = theta_ANKLE
    
    i+=1





