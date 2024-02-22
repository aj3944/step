from math import sin, cos,pi
from pylx16a.lx16a import *
import time


class Motor:
    center = 0
    def __init__(self,ID,cent,min_ang = 0,max_ang = 240):
        # self.I
        self.center = cent
        self.servo =  LX16A(ID)
        try:
            self.servo.set_id(ID)
            self.servo.set_angle_limits(min_ang,max_ang)
        except ServoTimeoutError as e:
            print(f"Servo {e.id_} is not responding. Exiting...")
    def move(self,offset =0 ):
        goal_pos = self.center + offset
        print(goal_pos)
        self.servo.move(goal_pos);
    def read(self):
        print(self.servo.get_last_instant_move_hw());
class Bot:

    def __init__(self):
        LX16A.initialize("/dev/ttyUSB0", 0.1)

        self.left_knee = Motor(1,136);
        self.left_thigh = Motor(2,105);
        self.left_hip = Motor(3,135);
        self.right_hip = Motor(4,127);
        self.right_thigh = Motor(5,95);
        self.right_knee = Motor(6,107);
        
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
            self.left_knee.read()
            self.left_thigh.read()
            self.left_hip.read()
            # self.right_knee.read()
            self.right_thigh.read()
            self.right_hip.read()
        except:
            print("ecnoder fail")


mark_4 = Bot()
mark_4.home()
mark_4.read()

mark_4.raise_foot(1,0)

# refresh = .5;


# t = 0
# tm = 0
# while 1:
#     if time.time() > (tm + refresh):
#         tm = time.time()
        # mark_4.raise_foot(0,sin(t) *4.);
#         # mark_4.raise_foot(1,sin(t) *4.);
# mark_4.raise_foot(0,0);
# mark_4.raise_foot(1,0);
# mark_4.left_knee.move(10);
# mark_4.left_thigh.move(-30);
# mark_4.left_hip.move(-10);
# mark_4.right_knee.move(0);
# mark_4.right_thigh.move(10);
# mark_4.right_hip.move(30);






