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
    #     goal_pos =te self.center + offset
    #     # print(goal_pos)
    #     self.servo.move(goal_pos);

    #     diff = self.center + offset - moved_to_last;


    def read(self):
        print(self.servo.get_last_instant_move_hw());

LX16A.initialize("/dev/ttyUSB0")

test_motor = Motor(33,220)
test_motor.servo.set_angle_limits(0, 240)
#test_motor.move(-1)
offsets = [-20, -10, 0, 10, 20, 0]

for x in offsets:
	test_motor.move(x)
	print(x)
	time.sleep(1)


