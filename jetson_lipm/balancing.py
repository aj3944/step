import numpy as np
import time
import accelerometer 

from bot import Bot



g = -9.81            # Acceleration due to gravity (m/s^2)
L = 0.290            # Length of pendulum (m)
m = 6                # Mass of pendulum (kg)
b = 0.5              # Damping coefficient
Kp = 100            # Proportional gain
Kd = 0.01             # Derivative gain
#Ki = 0.1            # Integral gain

device = accelerometer.Accelerometer()

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


mark_4 = Bot()
mark_4.home()
mark_4.read()
time.sleep(1)


# theta_dot_imu[0] = 0
i = 0;
while 1:
    i += 1 
    theta_dot_imu_x = tuple(device.readGyroData())[0]
    # u.append(0)
    # theta_dot.append(0)
    # theta.append(0)
    # '''
    # Angular vel of IMU in COM frame
    # '''
    # x = 0.04285
    # z = 0.00662

    # t_IMU_COM = np.array([x,0,z])
    # theta_dot_COM = np.array([theta_dot_imu_x, 0 , 0])+ np.cross(t_IMU_COM, np.array([0, 0 , theta_dot_imu_x]))

    # print(theta_dot_COM)


    # t_COM_GCOM = np.array([0,0,0.29])
    # theta_dotGCOM = np.array(theta_dot_COM) + np.cross(t_COM_GCOM, np.array(theta_dot_COM))

    # # print("Angular vel of gcom", theta_dotGCOM)

    # # if -0.05 < theta_dot_imu_x < 0.05 :
    # theta_dot_GCOM = np.append(theta_dot_GCOM,theta_dotGCOM)

    # # print("theta dot gcom",theta_dot_GCOM)
    # u[i] = m * L ** 2 * (theta_dot_GCOM[i] - theta_dot_GCOM[i - 1]) / dt

    # print("Torque value for GCOM",u[i])


    # theta_dot_dot_COM = g / L * np.sin(theta[i - 1]) + b / (m * L ** 2) * theta_dot[i - 1] + u[i - 1] / (m * L ** 2)

    # theta_dot[i] = theta_dot[i - 1] + dt * theta_dot_dot_COM
    # theta[i] = theta[i - 1] + dt * theta_dot[i]
    # # theta[i] *= 0.88
    # print("theta i :",theta[i])

    # # Position control
    # # ang_ctrl_GCOM = Kp * (theta_des - theta[i - 1]) + Kd * (theta_dot_des - theta_dot[i - 1])
    # # ang_ctrl_GCOM = Kp * (theta_des - theta[i - 1]) 

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


    # Supply motor ang_ctrl values
    # u[i] = Torque [Since dealing with angles] [If position control then u[i] would be Force]
    # # Check (IMU angl vel values) a

    # time.sleep(0.5)
    # print(u[i])

    # print(theta_dot_imu_x)