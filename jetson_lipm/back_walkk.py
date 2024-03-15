from math import sin, cos,pi
from  lx16a import *
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
        hip_pitch = 25 
        leg_footing = 0
        hip_footing = 3
        # LX16A.initialize("/dev/ttyUSB1")
        res = LX16A.initialize("/dev/ttyUSB0")
        # res = LX16A.initialize("/dev/ttyTHS1")
        print(res)
        self.left_knee = Motor(1,120 + leg_footing);
        self.left_thigh = Motor(2,80 + hip_pitch);
        self.left_hip = Motor(3,132 + hip_footing);
        self.right_hip = Motor(4,128 - hip_footing);
        self.right_thigh = Motor(5,122 - hip_pitch);
        self.right_knee = Motor(6,73 - leg_footing);
        
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
    x  = 17;

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

if __name__ == "__main__":

    mark_4 = Bot()
    mark_4.home()
    time.sleep(3)
    # mark_4.read()


    # mark_4.injest_ik([-4.299582831815598, 9.595166121368559, 4.288723309314223, 9.570967628619272])

    # traj_final_pos = [
    #     [-4.299582831815598, 9.595166121368559, 4.288723309314223, 9.570967628619272],
    #     [-4.299582831815598, 9.595166121368559, 14.785028134303765, 28.479995109408243],
    #     [-4.299582831815598, 9.595166121368559, -0.0, 0.0],
    #     [-14.798092362199307, 28.50976332213194, -0.0, 0.0],
    # ]
    # traj_final_pos = [
    #     [0,0,0,0],
    #     [0,0, 14.785028134303765, 28.479995109408243],
    #     [0,0, -0.0, 0.0],
    #     [-14.798092362199307, 28.50976332213194, -0.0, 0.0],
    #     [0,0, -0.0, 0.0],
    # ]
    # traj_final_pos = [
    #     [0,0,0,0],
    #     [-14,28, 14, -28],
    #     [0,0,0,0],
        # [0,0,0,0],
        # [0,0,0,0],
        # [0,0,0,0],
        # [-14, 28,0,0],
        # [0,0,0,0],
    # ]
    # 20 20 
    # traj_final_pos = [
    #     [0,0,0,0],
    #     [-20.749074809577262, 41.763596516716696, -0.0, -0.0],
    #     [-4.299456292148415, 9.594883503096952, -0.0, -0.0],
    #     [-4.299456292148415, 9.594883503096952, 20.734749216083888, -41.730199293516],
    #     [-4.299456292148415, 9.594883503096952, 4.2886506577562375, -9.570805364888873],
    #     [0,0,0,0],
    # ]

    # traj_final_pos =    [
    #     [-18.95221119919228, 35.4368390426368, -0.0, -0.0],
    #     [-4.299590703918358, 9.595183703208388, -0.0, -0.0],
    #     [-4.299590703918358, 9.595183703208388, 18.936930266458663, -35.40180552152586],
    #     [-4.299590703918358, 9.595183703208388, 4.288733652707215, -9.570990730086303],
    # ]

    # traj_final_pos =    [
    #     [-18.95221119919228, 35.4368390426368, -0.0, -0.0],
    #     [-4.299590703918358, 9.595183703208388, -0.0, -0.0],
    #     [-4.299590703918358, 9.595183703208388, 18.936930266458663, -35.40180552152586],
    #     [-4.299590703918358, 9.595183703208388, 4.288733652707215, -9.570990730086303],
    # ]
    # traj_final_pos = [
    #     [-34.79443542730939, 57.941060226143314, -0.0, -0.0],
    #     [-4.299518057178087, 9.595021451346701, -0.0, -0.0],
    #     [-4.299518057178087, 9.595021451346701, 34.763933947974174, -57.86928976762947],
    #     [-4.299518057178087, 9.595021451346701, 4.288659385189647, -9.570824857187665],
    # ]
    # traj_final_pos = [
    #     [14.78272163988919, -28.48165541758524, -0.0, -0.0],
    #     [4.277772231424706, -9.546591290519986, -0.0, -0.0],
    #     [4.277772231424706, -9.546591290519986, -14.793817069067156, 28.506794909830084],
    #     [4.277772231424706, -9.546591290519986, -4.2887915649850585, 9.57112023379839],
    # ]
# 
    # traj_final_pos = [
    #     [24.21233598852504, -40.064106160388725, -0.0, -0.0],
    #     [4.277724604737271, -9.546484917568748, -0.0, -0.0],
    #     [4.277724604737271, -9.546484917568748, -24.21970507408684, 40.08042370295624],
    #     [4.277724604737271, -9.546484917568748, -4.2886834523939985, 9.57087876956392],
    # ]

    traj_final_pos =  [
        [-18.936988630125086, 35.41234339799175, 0.0, 0.0],
        [0, 0, 0, 0],
        [0, 0, 18.947023817243757, -35.43517022819904],
        [0, 0, 0, 0],
    ]
    x  = 17;

    del_squat_A_t = -2.5;
    del_squat_A_f = 0;
    del_squat_B = 15;

    del_shift_L = -6;
    del_shift_R = 6;

    # del_shift_L = -0;
    # del_shift_R = 0;



    # traj_high_foot = [
    #     # [-del_squat_A, 2*del_squat_A, del_squat_A, -2*del_squat_A, 0.0 , 0.0],
    #     # [-del_squat_B, 2*del_squat_B, del_squat_B, -2*del_squat_B, 0.0 , 0.0],
    #     # [1*-del_squat_A_t, 1*2*del_squat_A_f, 1*del_squat_A_t, 1*-2*del_squat_A_f, del_shift_L , del_shift_R],
    #     # [1*-del_squat_A, 1*2*del_squat_A, 1*del_squat_A, 1*-2*del_squat_A, del_shift_L , del_shift_R],
    #     # [1*-del_squat_A, 1*2*del_squat_A, 1*del_squat_A, 1*-2*del_squat_A, del_shift_L , del_shift_R],
    #     # [2*-del_squat_A, 1*2*del_squat_A, 2*del_squat_B, 1*-2*del_squat_B, del_shift_L , del_shift_R],
    #     # [1*-del_squat_A, 1*2*del_squat_A, 1*del_squat_B, 1*-2*del_squat_B, del_shift_L , del_shift_R],
    #     [-x + del_squat_A_t, 2*x + del_squat_A_f , 0.0, 0.0, del_shift_L , del_shift_L],
    #     # [-17.061858301371288, 35.87581869868777, 0.0, 0.0, 0.0 , -0.0],
    #     # [-0.0, 0.0, 0.0, 0.0, 0.0 , 0.0],
    #     # [-0.0, 0.0, 0.0, 0.0, 0.0 , 0.0],
    #     # [1*-del_squat_B, 1*2*del_squat_B, 1*del_squat_B, 1*-2*del_squat_B, del_shift_L , del_shift_R],
    #     # [-0.0, 0.0, 0.0, 0.0, 0.0 , del_shift_L , del_shift_R],
    #     [-0.0, 0.0, 0.0, 0.0, 0.0 , 0.0],
    #     # [-0.0, 0.0, 0.0, 0.0, 0.0 , 0.0],
    #     # [-0.0, 9.546566366780933, 15.655523569481783, -28.148578158772242, 2.0 , 0.0],
    #     [-0.0, 0.0, x - del_squat_A_t, -2*x -del_squat_A_f, del_shift_R , del_shift_R],
    #     [-0.0, 0.0, 0.0, 0.0, 0.0 , 0.0],
    #     # [-0.0, 9.546566366780933, 4.288755587337035, -9.57103987946287, 0.0 , 0.0],
    # ]

    traj_high_foot = step_traj(0,0);
    traj_high_foot.extend(step_traj(0));
    for i in range(40): 
        traj_high_foot.extend(step_traj(-2,-2));
    # for i in range(len(traj_high_foot)):
    #     traj_high_foot[i][1] = 90;
    #     traj_high_foot[i][3] = -90;

    # traj_final_pos = [
    # [-28.99699862064352, 50.4049533326476, -0.0, -0.0],
    # [4.2777260441458065, -9.546488132449735, -0.0, -0.0],
    # # [4.2777260441458065, -9.546488132449735, -0.0, -0.0],
    # # [4.2777260441458065, -9.546488132449735, -0.0, -0.0],
    # [4.2777260441458065, -9.546488132449735, 29.000213531330907, -50.4118660936847],
    # [4.2777260441458065, -9.546488132449735, -4.288680843518162, 9.570872942766313],
    # ]


    # for x in range(100,300):
    for t4,i in zip(traj_high_foot,range(len(traj_high_foot))):
        mark_4.injest_ik(t4,0.0200)
                # mark_4.injest_ik(t4,0.01)
        #     mark_4.injest_ik(t4,0.065)
        # else:
        #     mark_4.injest_ik(t4,0.125)            


        # if i == 1:
        #     time.sleep(0.125)

# # #         # if i%2 == 0:
    #         # if i%2 == 1:

    #         mark_4.injest_ik(t4,0.065)
            # mark_4.injest_ik_delay(t4,150)
            # time.sleep(.7)

# mark_4.read()

# mark_4.raise_foot(1,0)

# refresh = .5;


# t = 0
# tm = 0
# while 1:
#     if time.time() > (tm + refresh):
#         tm = time.time()
#         mark_4.raise_foot(0,sin(t) *4.);
#         # mark_4.raise_foot(1,sin(t) *4.);
# mark_4.raise_foot(0,0);
# mark_4.raise_foot(1,0);
# mark_4.left_knee.move(10);
# mark_4.left_thigh.move(-30);
# mark_4.left_hip.move(-10);
# mark_4.right_knee.move(0);
# mark_4.right_thigh.move(10);
# mark_4.right_hip.move(30);






