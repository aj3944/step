


import imu 
from step_bot import Bot
import time
from math import sin,cos,atan2,copysign


IMU = imu.Accelerometer()

mark_4 = Bot()
mark_4.home()
time.sleep(3)

sway = 3.5;



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

peak_x = 0;
peak_y = 0;

maxed_x = True;
maxed_y = True;


fx_max = 0;
fy_max = 0;


while not stop:
	time_now = time.time()
	accs = list(IMU.readAccData())
	gyros = [x for x in IMU.readGyroData()]

	# print(gyros)
	if abs(gyros[0]) > abs(peak_x):
		peak_x = gyros[0]
		maxed_x = False;
		fx_max = accs[0]
	else:
		maxed_x = True;
	if abs(gyros[1]) > abs(peak_y):
		peak_y = gyros[1]
		maxed_y = False;
		fy_max = accs[1]
	else:
		maxed_y = True;



	stable_x = abs(gyros[0]) <= 0.02;
	stable_y = abs(gyros[1]) <= 0.02;

	# peak_x = abs(gyros[0]) <= 0.02;
	# peak_y = abs(gyros[1]) < 0.02;

	if stable_x and stable_y:
		peak_x = 0;
		peak_y = 0;
		fx_max = 0;
		fy_max = 0;
		if time_old + time_delta < time_now:
			mark_4.injest_ik_delay(traj_list[traj_index],500);
			time_old = time.time()
			traj_index += 1;
			traj_index %= N;
	else:
		if maxed_x and maxed_y:
			print("FALL START",end='\t');
			print(gyros,end='\t');
			print(accs);
			# mark_4.injest_ik_delay([0,0,0,0,0,0],20);
		else:
			continue
		kick_l = 0;
		kick_r = 0;
		roll_y = 0;
		step_l = 0;
		step_r = 0;

		# if not stable_x:  
			# if gyros[0] < 0:
		kick_l = 0
		kick_r = -0
		left_leg_lift = 0;
		right_leg_lift = 0;
			# else:
			# 	kick_l = 20
			# 	kick_r = -20
		# if not stable_y:
				# marignal_toq = copysign(1,gyros[0])
		step_len = 5;
		step_height = 15;

		if gyros[1] > 0:
			print("stepping back")
			if gyros[2] < 0 :
				print("left leg lift ")
				left_leg_lift = step_height;
				step_l = step_len;
			else:
				print("right leg lift ")
				right_leg_lift = step_height;
				step_r = -step_len;
		else:
			print("stepping forward")
			if gyros[2] > 0 :
				print("left leg lift ")
				left_leg_lift = step_height;
				step_l = -step_len;
			else:
				print("right leg lift ")
				right_leg_lift = step_height;
				step_r = step_len;


		# if gyros[2] > 0 :
		# # 	roll_y = 15;
		# 	print("left leg lift ")
		# 	left_leg_lift = 10;
		# 	# right_leg_lift = 5;
		# 	#kick_l = 0;
		# 	#step_l = 0;
		# else:
		# # 	roll_y = -15;
		# 	print("right leg lift")
		# 	# left_leg_lift = 5;
		# 	right_leg_lift = 10;
		# 	# step_r = 0;

		# if gyros[1] > 0:
		# 	print("step back")
		# else:
		# 	print("step fwd")


		mark_4.injest_ik_delay([step_l - left_leg_lift,2*left_leg_lift,- step_r + right_leg_lift,-2*right_leg_lift,kick_l,kick_r],50);
		# stop = True
		time.sleep(.2)
		# mark_4.injest_ik_delay([0 ,0,-0 - 0,0,kick_l,kick_r],200);
		# time.sleep(1)


		mark_4.injest_ik_delay([0,0,0,0,0,0],200);
		time.sleep(.1)			
		# if marignal_toq > 0:
		# 	# Right step 
		# 	mark_4.injest_ik_delay([roll_y,0,-roll_y,0,kick_l,kick_r],20);
		# else:
			# Left step 
	# print(gyros)