from math import sin, cos,pi,atan2
from lx16a import *
import time
import imu 

class Motor:
    center = 0
    moved_last_offset = 0;
    def __init__(self,ID,cent,min_ang = 0,max_ang = 240):
        self.center = cent
        try:
            self.servo =  LX16A(ID)
        except Exception as e:
            print(e)
    def move(self,offset =0 ):
        goal_pos = self.center + offset;
        self.moved_last_offset = offset 
        self.servo.move(goal_pos);
    def moveIn(self,offset =0 , ms_to_move = 500):
        goal_pos = self.center + offset;
        self.moved_last_offset = offset 
        self.servo.move(goal_pos,ms_to_move);
    def read(self):
        print(self.servo.get_last_instant_move_hw());

class Bot:
    IMU = False
    acc =  [ 0,0,0]
    gyro = [ 0,0,0]
    atilts = [0,0]
    gtilts = [0,0]
    int_time = 0
    Q_degress = [0,0,0,0,0,0]
    def __init__(self):
        LX16A.initialize("/dev/ttyTHS1")
        self.IMU = imu.Accelerometer()
        hip_pitch = 35 
        hip_offset = -1;
        leg_footing = 0;
        hip_footing = 3
        self.left_knee = Motor(11,124 + leg_footing*2);
        self.left_thigh = Motor(12,82 + hip_pitch +        leg_footing);
        self.left_hip = Motor(13,132 + hip_footing + hip_offset);
        self.right_hip = Motor(24,128 - hip_footing + hip_offset);
        self.right_thigh = Motor(25,123 - hip_pitch -  leg_footing);
        self.right_knee = Motor(26,73 - leg_footing*2);
        self.readIMU()
    def home(self):
        self.left_knee.move()
        self.left_thigh.move()
        self.left_hip.move()
        self.right_knee.move()
        self.right_thigh.move()
        self.right_hip.move()
    def readIMU(self):
        if int_time == 0:
            prev_time = time.time()
        else:
            prev_time = int_time
        int_time = time.time()
        delta_t = int_time - prev_time;

        self.acc = list(self.IMU.readAccData())
        self.gyro = list(self.IMU.readGyroData())
        tx = atan2(self.acc[0],self.acc[2])
        ty = atan2(self.acc[1],self.acc[2])
        self.atilts = [tx,ty]
        self.gtilts = [self.gtilts[0] + delta_t*self.gyro[0],self.gtilts[1] + delta_t*self.gyro[1]]
    def raise_foot(self,foot ,height):
        if foot == 0:
            self.left_knee.move(height);
        else:
            self.right_knee.move(height);
    def swing_hips(self,ang):
        n_traj = self.Q_degress;
        n_traj[5] += ang;
        n_traj[6] += ang;
        self.injest_ik(n_traj)
    def read(self):
        try:
            self.right_knee.read()
            time.sleep(0.01)
            self.left_knee.read()
            time.sleep(0.01)
            self.left_thigh.read()
            time.sleep(0.01)
            self.left_hip.read()
            time.sleep(0.01)
            self.right_thigh.read()
            time.sleep(0.01)
            self.right_hip.read()
            time.sleep(0.01)
        except Exception as e:
            print(e)
            print("ecnoder fail")
    def injest_ik(self,traj_4,time_to_execute = 1):
        if len(traj_4) == 4:
            traj_4.append(0);
            traj_4.append(0);
        self.Q_degress = traj_4;
        print("injecting traj",traj_4)
        init_left_knee_last_offset = self.left_knee.moved_last_offset
        init_left_thigh_last_offset = self.left_thigh.moved_last_offset
        init_left_hip_last_offset = self.left_hip.moved_last_offset
        init_right_knee_last_offset = self.right_knee.moved_last_offset
        init_right_thigh_last_offset = self.right_thigh.moved_last_offset
        init_right_hip_last_offset = self.right_hip.moved_last_offset


        delta_left_knee =  traj_4[1] - self.left_knee.moved_last_offset;
        delta_left_thigh =  traj_4[0] - self.left_thigh.moved_last_offset;
        delta_left_hip =  traj_4[4] - self.left_hip.moved_last_offset;
        delta_right_knee =  traj_4[3] - self.right_knee.moved_last_offset;
        delta_right_thigh =  traj_4[2] - self.right_thigh.moved_last_offset;
        delta_right_hip =  traj_4[5] - self.right_hip.moved_last_offset;


        delta_left_knee *= 1/(1000*time_to_execute)
        delta_left_thigh *= 1/(1000*time_to_execute)
        delta_right_knee *= 1/(1000*time_to_execute)
        delta_right_thigh *= 1/(1000*time_to_execute)
        delta_left_hip *= 1/(1000*time_to_execute)
        delta_right_hip *= 1/(1000*time_to_execute)

        for i in range(int(1000*time_to_execute)):
            self.left_knee.move(init_left_knee_last_offset + delta_left_knee*i)
            self.left_thigh.move(init_left_thigh_last_offset + delta_left_thigh*i)
            self.right_knee.move(init_right_knee_last_offset + delta_right_knee*i)
            self.right_thigh.move(init_right_thigh_last_offset + delta_right_thigh*i)
            self.left_hip.move(init_left_hip_last_offset + delta_left_hip*i)
            self.right_hip.move(init_right_hip_last_offset + delta_right_hip*i)
    def injest_ik_delay(self,traj_4,time_to_execute = 500):
        print("injecting traj",traj_4)
        self.left_knee.moveIn(traj_4[1],time_to_execute)
        self.left_thigh.moveIn(traj_4[0],time_to_execute)
        self.right_knee.moveIn(traj_4[3],time_to_execute)
        self.right_thigh.moveIn(traj_4[2],time_to_execute)
