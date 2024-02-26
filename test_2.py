from pylx16a.lx16a import *
import time

LX16A.initialize("/dev/ttyUSB0")



for i in range(1,7):
	try:
		motor1 = LX16A(i);
		# motor1.set_id(ID)
		motor1.set_angle_limits(0,240)

		motor1.move(110)
		time.sleep(1)
	except Exception as e:
		print(e,i)

# motor1 = LX16A(2);
# # motor1.set_id(ID)
# motor1.set_angle_limits(0,240)

# motor1.move(120)