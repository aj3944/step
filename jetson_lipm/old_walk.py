from math import sin, cos,pi
from lx16a import *
import time

class Motor:
    center = 0
    moved_last_offset = 0;

    def __init__(self,ID,cent,min_ang = 0,max_ang = 240):
        # self.I
        self.center = cent
        try:
            self.servo =  LX16A(ID)
        # try:
        #     pass
        #     # self.servo.set_id(ID)
        #     # self.servo.set_angle_limits(min_ang,max_ang)
        except Exception as e:
            print(e)
            # print(f"Servo {e.id_} is not responding. Exiting...")
    def move(self,offset =0 ):
        goal_pos = self.center + offset;
        self.moved_last_offset = offset 
        # print(goal_pos)
        self.servo.move(goal_pos);
    def moveIn(self,offset =0 , ms_to_move = 500):
        goal_pos = self.center + offset;
        self.moved_last_offset = offset 
        # print(goal_pos)
        self.servo.move(goal_pos,ms_to_move);
    # def moveIn(self,offset =0 ,time_to_move = 1):
    #     goal_pos = self.center + offset
    #     # print(goal_pos)
    #     self.servo.move(goal_pos);

    #     diff = self.center + offset - moved_to_last;


    def read(self):
        print(self.servo.get_last_instant_move_hw());
class Bot:

    def __init__(self):

        # LX16A.initialize("/dev/ttyTHS1")
        LX16A.initialize("/dev/ttyUSB0")
        hip_pitch = 40 
        hip_offset = -1;
        leg_footing = 0;
        hip_footing = 3
        self.left_knee = Motor(11,124 + leg_footing*2);
        self.left_thigh = Motor(12,82 + hip_pitch +        leg_footing);
        self.left_hip = Motor(13,132 + hip_footing + hip_offset);
        self.right_hip = Motor(24,128 - hip_footing + hip_offset);
        self.right_thigh = Motor(25,123 - hip_pitch -  leg_footing);
        self.right_knee = Motor(26,73 - leg_footing*2);
        
    def home(self):
        self.left_knee.move()
        self.left_thigh.move()
        self.left_hip.move()
        self.right_knee.move()
        self.right_thigh.move()
        self.right_hip.move()
    def raise_foot(self,foot ,height):
        if foot == 0:
            self.left_knee.move(height);
            # self.left_thigh.move(height);
        else:
            self.right_knee.move(height);
            # self.right_thigh.move(height);
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

if __name__ == "__main__":

    mark_4 = Bot()
    mark_4.home()
    time.sleep(3)

    traj_final_pos =  [
        [-18.936988630125086, 35.41234339799175, 0.0, 0.0],
        [0, 0, 0, 0],
        [0, 0, 18.947023817243757, -35.43517022819904],
        [0, 0, 0, 0],
    ]
    # x  = 5;
    x  = -25;
    x_prime  = 10;
    y = 4;
    # y = 10;

    y_right =  4;
    y_left =  4;
    traj_walk_foot = [];

    llstep = -5;

    traj_walk_foot.append([0,0,0,0,0,0])
    # for i in range(10):
    #     traj_walk_foot.append([0,0,0,0,i*y_right/10,i*y_right/10])
    traj_walk_foot.append([0,0,0,0,y_right,y_right])

    traj_walk_foot.append([-llstep,0,-llstep,0,0,0])
    # for i in range(10):
    #     traj_walk_foot.append([0,0,0,0,-i*y_left/10,-i*y_left/10])
    traj_walk_foot.append([0,0,0,0,-y_left,-y_left])

    traj_walk_foot.append([llstep,0,llstep,0,0,0])
    traj_walk_foot.append([0,0,0,0,0,0])

    traj_high_foot = [
        # [-0, 0, 0.0, 0.0, -y , -y],
        [0, 0, 0.0, 0.0, y , y],
        # [x, 0, 0.0, 0.0, y , y],
        [x, -2*x, -x_prime, 0.0, y , y],
        # [-0.0, 0.0, 0.0, 0.0, 0.0 , 0.0],
        # [-2*x, 0, -x, 0, y, y],
        # [-0.0, 0.0, -x, -2*x, y, y],
        # [-0.0, 0.0, x, -2*x, y, y],
        [-0.0, 0.0, 0.0, 0.0, 0.0 , 0.0],
        # [-0.0, 0.0, 0.0, 0.0, 0.0 , 0.0],
        [0, 0, 0.0, 0.0, -y , -y],
        [x_prime, 0, -x, 2*x, -y , -y],
        [-0.0, 0.0, 0.0, 0.0, 0.0 , 0.0],
    ]
    speeds = [
        0.08,
        0.02,
        0.01,
        0.04,
        0.04,
        0.02,
        0.01,
        0.01,
    ]
    # traj_high_foot = [
    #     [-x, 2*x, 0.0, 0.0, -y , -y],
    #     # [-2*x, 0, -x, 0.0, -y , -y],
    #     # [-0.0, 0.0, 0.0, 0.0, 0.0 , 0.0],
    #     # [-2*x, 0, -x, 0, y, y],
    #     # [-0.0, 0.0, -x, -2*x, y, y],
    #     [-0.0, 0.0, 0.0, 0.0, 0.0 , 0.0],
    #     [-0.0, 0.0, x, -2*x, y, y],
    #     [-0.0, 0.0, 0.0, 0.0, 0.0 , 0.0],
    # ]


    new_traj = []

    # new_traj.append([])
    # new_traj.append([0,0,0,0,0,0])


    f = -20;
    new_traj.append([f,-2*f,0,0,0,0])
    # new_traj.append([0,0,-f,2*f,0,0])
    # new_traj.append([f,0,-f,0,0,0])
    # new_traj.append([-f,0,f,0,0,0])
    # new_traj.append([0,0,0,0,f,f])
    # new_traj.append([0,0,0,0,-f,-f])


    for i in range(4):
        for t4,i in zip(traj_final_pos,range(len(traj_final_pos))):
                mark_4.injest_ik(t4,0.019)





