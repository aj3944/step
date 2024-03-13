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
m = 2               # Mass of pendulum (kg)
b = 2             # Damping coefficient
Kp = 10            # Proportional gain
Ki = 0.1          # Integral gain


# Initial conditions
# theta_COM_init = 0.1745     #10 degrees # Initial angle (radians)
# theta_dot_COM_init = 0                # Initial angular velocity (radians/s)


theta_COM_prev = 0.175 #10degs
theta_dot_COM_prev = 0

theta_ANKLE_prev = theta_COM_prev
angl_vel_ctrl_ANKLE_prev = 0

integral = 0

u = []
u.append(0)

# Simulation parameters
dt = 0.01                        # Time step (s)
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
ANKLE.translation = [0,0.045,0]
ANKLE.rotation = R.from_matrix([[0, -1, 0],[0,0,-1],[1,0,0]])

HIP.rotation = R.from_rotvec([0., control_angle, 0., ])
HIP.translation = [0,0,L]


IMU_TO_HIP = HIP * ANKLE * GCOM * COM * IMU
IMU_TO_COM = COM * IMU

IMU_TO_GCOM =  GCOM * COM * IMU
IMU_TO_ANKLE = ANKLE * IMU_TO_GCOM
COM_TO_ANKLE = ANKLE * GCOM * COM

# print(IMU_TO_HIP.T_matrix())
# print(IMU_TO_ANKLE.T[:3,-1])
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

def angular_vel_transform(angular_vel , transformation):

    # print("Translation vector", translation_vector)
    # print("Angular_velocity",angular_vel)
    theta_dot_transform = np.dot(transformation.T[:3,:3],np.array(angular_vel))+ np.cross(transformation.T[:3,-1] , np.array(angular_vel))
    
    theta_dot_transform = theta_dot_transform.tolist()
    # print("Theta_dot_transform",theta_dot_transform)

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
    theta_dot_imu = tuple(device.readGyroData())

    print("theta_dot_imu" , theta_dot_imu)
    
    theta_dot_IMU_TO_COM = angular_vel_transform(theta_dot_imu, IMU_TO_COM)

    # theta_dot_IMU_TO_ANKLE = angular_vel_transform(theta_dot_IMU_TO_COM , IMU_TO_ANKLE )

    # print("theta_dot_IMU_TO_ANKLE",theta_dot_IMU_TO_ANKLE)

    '''
    Calculated Angular accelereation of the COM
    '''

    theta_dot_dot_COM = ((g / L ) * np.sin(theta_COM_prev)) + ((b / (m * (L ** 2)))* theta_dot_COM_prev) + (u[i] / (m  * (L**2) ) )

    print("theta_dot_dot_COM",theta_dot_dot_COM)

    theta_dot_COM = theta_dot_COM_prev + (dt * theta_dot_dot_COM)
    theta_dot_COM_prev = theta_dot_COM
    print("np.sintheta = ",np.sin(theta_COM_prev))
    print("theta_dot_COM",theta_dot_COM_prev)
    print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")

    thetaCOM = theta_COM_prev + (dt * theta_dot_COM)
    theta_COM_prev = thetaCOM

    # theta_dot_COM_TO_ANKLE = angular_vel_transform([theta_dot_COM,0,0], COM_TO_ANKLE)

    # print("theta_dot_COM_TO_ANKLE",theta_dot_COM_TO_ANKLE)

    # print("Norm of theta_dot_COM_TO_ANKLE",np.linalg.norm(theta_dot_COM_TO_ANKLE))

    # print("Norm of theta_dot_IMU_TO_ANKLE",np.linalg.norm(theta_dot_IMU_TO_ANKLE))

    # print("Difference between theta_dot", (np.linalg.norm(theta_dot_COM_TO_ANKLE) - (np.linalg.norm(theta_dot_IMU_TO_ANKLE) )))

    # tau = (m * (L** 2 )) * ((np.linalg.norm(theta_dot_COM_TO_ANKLE) - (np.linalg.norm(theta_dot_IMU_TO_ANKLE))) / dt)

    # tau = (m * (L** 2 )) * (((0-np.linalg.norm(theta_dot_IMU_TO_COM))) / dt)

    tau = (m * (L** 2 )) * ((0 - angl_vel_ctrl_ANKLE_prev) / dt)
    print("Torque supplied based on error correction \t" , tau)

    u.append(tau)

    '''
    PI control for Ankle
    '''
    theta_error = theta_des - theta_ANKLE_prev

    integral += theta_error * dt

    ang_ctrl_ANKLE = Kp* (theta_error) + Ki * (integral)
    ang_ctrl_ANKLE_prev = ang_ctrl_ANKLE

    angl_vel_ctrl_ANKLE = (angl_vel_ctrl_ANKLE -  ang_ctrl_ANKLE_prev) /dt

    print("Radians at the Ankle \t", ang_ctrl_ANKLE)

    # ang_ctrl_ankle= Kp * (theta_des - theta[i - 1]) + Kd * (theta_dot_des - theta_dot[i - 1])

    hip_motor = np.rad2deg(ang_ctrl_ANKLE)

    print("Hip  motor angle supplied\t" , hip_motor)

    
    # mark_4.injest_ik([0,0,0,0,-hip_motor,-hip_motor])

    '''
    Integrating theta_dot_IMU_TO_ANKLE
    '''

    theta_ANKLE= theta_ANKLE_prev+ (dt * np.linalg.norm(theta_dot_IMU_TO_COM))
    
    theta_ANKLE_prev = theta_ANKLE
    
    i+=1







