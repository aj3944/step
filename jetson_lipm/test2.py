from math import sin, cos,pi
from  lx16a import *
import time





class Motor:
    center = 0
    moved_last_offset = 0;

    def __init__(self,ID,cent,min_ang = 0,max_ang = 240):
        # self.I
        self.center = cent
        # try:
        self.servo =  LX16A(ID);
            # for i in range(10000000):
            #     pass
        # try:
        #     pass
        #     # self.servo.set_id(ID)
        #     # self.servo.set_angle_limits(min_ang,max_ang)
        # except Exception as e:
            # print(e)
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
        hip_pitch = 45 
        leg_footing = 1
        hip_footing = 2
        # LX16A.initialize("/dev/ttyUSB1")
        res = LX16A.initialize("/dev/ttyUSB0")
        # res = LX16A.initialize("/dev/ttyTHS1")
        print(res)
        err = True;
        while err:
            try:
                self.left_knee = Motor(11,124 + leg_footing*2);
                self.left_thigh = Motor(12,80 + hip_pitch +        leg_footing);
                self.left_hip = Motor(13,132 + hip_footing);
                self.right_hip = Motor(24,128 - hip_footing);
                self.right_thigh = Motor(25,128 - hip_pitch -  leg_footing);
                self.right_knee = Motor(26,73 - leg_footing*2);
                err = False
            except Exception as e:
                print(e)
                err = True
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

def step_traj(step_len_l = 0.1,step_len_r = 0.1):
    x  = 10;

    del_squat_A_t_l = -1*step_len_l;
    del_squat_A_t_r = -1*step_len_r;
    del_squat_A_f = 0;
    del_squat_B = 15;

    del_shift_L = -6;
    del_shift_R = 6;

    # del_shift_L = -0;
    # del_shift_R = 0;



    traj_high_foot = [
        [-x + del_squat_A_t_l, 2*x + del_squat_A_f , 0.0, 0.0, del_shift_L , del_shift_L],
        [-0.0, 0.0, 0.0, 0.0, 0.0 , 0.0],
        [-0.0, 0.0, x - del_squat_A_t_r, -2*x -del_squat_A_f, del_shift_R , del_shift_R],
        [-0.0, 0.0, 0.0, 0.0, 0.0 , 0.0],
    ]  
    return traj_high_foot
def smart_step_traj(step_len_l = 0.1,step_len_r = 0.1):
    xl  = step_len_l;
    xr = step_len_r;
    del_shift_L = -5;
    del_shift_R = 5;
    # traj_high_foot = [
    #     [-0 , 0 , 0.0, 0.0,del_shift_R, del_shift_R],
    #     # [-0 , 2*xl , 0.0, 0.0,del_shift_R, del_shift_R],
    #     [-xl , 2*xl , 0, 0.0,del_shift_R, del_shift_R],
    #     # [-0.0, 0.0, 0, -0, del_shift_L , del_shift_L],
    #     # [-0.0, 0.0, -xr, 0, 0.0 , 0.0],
    #     [-xl , 0 , -xr, 0.0,del_shift_L, del_shift_L],
    #     # # [-0 , 0 , 0.0, -2*xr,del_shift_L, del_shift_L],
    #     # [-0 , 0 , xr, -2*xr,del_shift_L*2, del_shift_L*2],
    #     # # [-0.0, 0.0, 0, -0, del_shift_R , del_shift_R],
    #     [-0.0, 0.0, 0, 0, 0.0 , 0.0],
    # ]  
    traj_high_foot = [
        [0 , 0 , 0.0, 0.0,del_shift_R, del_shift_R],
        # [-xl , xl , xr, 0.0,del_shift_R, del_shift_R],
        [-xl , 2*xl , 0, 0.0,del_shift_R, del_shift_R],
        # [-0 , 2*xl , 0.0, 0.0,0, del_shift_R],
        # [-xl , 2*xl , 0, 0.0,del_shift_R, del_shift_R],
        # [-0.0, 0.0, 0, -0, del_shift_L , del_shift_L],
        # [-0.0, 0.0, -xr, 0, 0.0 , 0.0],
        # [-xl , 0 , -xr, 0.0,del_shift_L, del_shift_L],

        # [-0 , 0 , 0.0, 0,del_shift_R, del_shift_R],


        # [-xl , 0, 0, 0.0,del_shift_R, del_shift_R],
        [0, 0, 0, 0,del_shift_L, del_shift_L],
        [0, 0, xr, -2*xr,del_shift_L, del_shift_L],


        # [-0 , 0 , xr, -2*xr,del_shift_L, 0],

        # [xl , 0 , xr, 0.0,del_shift_R, del_shift_R],


        # # [-0.0, 0.0, 0, -0, del_shift_R , del_shift_R],
        # [-0.0, 0.0, 0, 0, 0.0 , 0.0],
        # [-0.0, 0.0, 0, 0, 0.0 , 0.0],
        # [-0.0, 0.0, 0, 0, 0.0 , 0.0],
        # [-0.0, 0.0, 0, 0, 0.0 , 0.0],
        # [-0.0, 0.0, 0, 0, 0.0 , 0.0],
        # [-0.0, 0.0, 0, 0, 0.0 , 0.0],
    ]  
    return traj_high_foot

# traj_walk_foot = []
# x  = 10;
# y_right =  5;
# y_left =  7;
# traj_walk_foot.append([0,0,0,0,0,0])
# for i in range(10):
#     traj_walk_foot.append([0,0,0,0,i*y_right/10,i*y_right/10])
# traj_walk_foot.append([0,5,0,-5,y_right,y_right])
# traj_walk_foot.append([0,0,0,0,0,0])
# for i in range(10):
#     traj_walk_foot.append([0,0,0,0,-i*y_left/10,-i*y_left/10])
# # traj_walk_foot.append([0,7,0,-7,-y_left,-y_left])

# traj_walk_foot.append([0,0,0,0,0,0])
# traj_walk_foot.append([0,0,0,0,0,0])
# traj_walk_foot.append([0,0,0,0,0,0])
# traj_walk_foot.append([0,0,0,0,0,0])

def realsense_ang(ang):
    return [ang,0,-ang,0,0,0]


def exec_traj(traj,speed = 0.018):
    for t4,i in zip(traj,range(len(traj))):
        mark_4.injest_ik(t4,speed)    

if __name__ == "__main__":

    mark_4 = Bot()
    mark_4.home()
    time.sleep(3)
    # mark_4.read()
    traj_high_foot = [];
    traj_move_jetson = [];


    for i in range(100):
        traj_move_jetson.append(realsense_ang(30))
        # traj_move_jetson.append(realsense_ang(20))
        traj_move_jetson.append(realsense_ang(0))
        traj_move_jetson.append(realsense_ang(-30))
    # traj_high_foot.extend(step_traj(0,0));
    for i in range(2): 
        traj_high_foot.extend(smart_step_traj(6,6));
    traj_high_foot.append([0,0,0,0,0,0])

    # exec_traj(traj_move_jetson,0.2);
    exec_traj(traj_high_foot,0.025);

                # mark_4.injest_ik(t4,0.01)

