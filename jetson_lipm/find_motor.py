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
            # print(e)
            raise(e)
            # pass
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


LX16A.initialize("/dev/ttyUSB0")

for i in range(300):
    try:
        test_motor = Motor(i,90);
        print(i)
        time.sleep(0.01)
    except:
        pass