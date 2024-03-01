from pylx16a.lx16a import *
import time

LX16A.initialize("/dev/ttyUSB0")

# for j in range(255):
# 	try:
# 		motor1 = LX16A(1);
# 		# motor1.set_id(1)
# 		# motor1.servo_mode()
# 		# motor1.disable_torque()
# 		print()
# 		print(dir(motor1))
# 		motor1.set_angle_limits(0,240)
# 		# motor1.enable_torque()
# 		for i in range(10,220):
# 			motor1.move(i)
# 			time.sleep(.01)
# 	except Exception as e:
# 		print(e,i)
# 	try:
# 		motor1 = LX16A(i);
# 		# motor1.set_id(ID)
# 		motor1.set_angle_limits(0,240)

# 		motor1.move(40)
# 		time.sleep(1)
# 	except Exception as e:
# 		print(e,i)

# motor1 = LX16A(2);
# # motor1.set_id(ID)
# motor1.set_angle_limits(0,240)

# motor1.move(120)

motor1 = LX16A(1);
# motor1.set_id(1)
# motor1.servo_mode()
# motor1.disable_torque()
print()
print(dir(motor1))
motor1.set_angle_limits(0,240)
# motor1.enable_torque()
while 1:
	for i in range(0,240):
		motor1.move(i)
		# time.sleep(.05)