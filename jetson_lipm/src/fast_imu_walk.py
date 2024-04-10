


import imu 
from step_bot import Bot
import time
from math import sin,cos,atan2,copysign


IMU = imu.Accelerometer()

mark_4 = Bot()
mark_4.home()
time.sleep(3)

sway = 0.5;



traj_list = [
	[ 0, 0, 0, 0, sway, sway],
	[ 0, 0, 0, 0, -sway, -sway],
]


N = len(traj_list)

traj_index = 0

motor_update_rate = 2 #Hz

time_old = time.time();
time_delta = 1/motor_update_rate;

max_vals = 20;


stop = False;



falling = False;
fall_left = False;
# fall_right = False;
stable_x = True
stable_y = True
while not stop:
	time_now = time.time()
	accs = list(IMU.readAccData())
	gyros = [x for x in IMU.readGyroData()]
	stable_x = abs(gyros[0]) <= 0.02;
	# stable_y = abs(gyros[1]) < 0.02;
	print(gyros)

	if stable_x and stable_y:
		if time_old + time_delta < time_now:
			mark_4.injest_ik_delay(traj_list[traj_index],500);
			time_old = time.time()
			traj_index += 1;
			traj_index %= N;
	else:
		kick_l = 0;
		kick_r = 0;
		roll_y = 0;
		step_l = 0;
		step_r = 0;

		if not stable_x:  
			# if gyros[0] < 0:
			kick_l = 20
			kick_r = -20
			# else:
			# 	kick_l = 20
			# 	kick_r = -20
		if not stable_y:
			marignal_toq = copysign(1,gyros[0])
			if gyros[1] < 0:
				roll_y = 15;
			else:
				roll_y = -15;
		mark_4.injest_ik_delay([roll_y,0,-roll_y,0,kick_l,kick_r],100);
		# stop = True
		time.sleep(.2)
		mark_4.injest_ik_delay([0,0,0,0,0,0],200);
		time.sleep(1)			
		# if marignal_toq > 0:
		# 	# Right step 
		# 	mark_4.injest_ik_delay([roll_y,0,-roll_y,0,kick_l,kick_r],20);
		# else:
			# Left step 
	# print(gyros)