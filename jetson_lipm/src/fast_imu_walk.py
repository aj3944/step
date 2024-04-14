


import imu 
from step_bot import Bot
import time
from math import sin,cos,atan2,copysign,pi
from scipy.spatial.transform import Rotation as R
from quaternion import Transformation
from mad_ahrs import MadgwickAHRS



IMU = imu.Accelerometer()

mark_4 = Bot()
mark_4.home()
time.sleep(3)

sway = 2;

sh =  18;
sl = 2;
sb = 0;

traj_list = [
	[ 0, 0, 0, 0, sway, sway],
	[ -sh + sb, sh*2-sl, sb, 0, sway, sway],
	[0,0,0,0,0,0],
	[ 0, 0, 0, 0, -sway, -sway],
	[ -sb, 0, sh - sb, -sh*2+sl, -sway, -sway],
	[0,0,0,0,0,0],
]


N = len(traj_list)

traj_index = 0

motor_update_rate = 11.1 #Hz

time_old = time.time();
time_delta = 1/motor_update_rate;
time_ms = int(time_delta*1000);

max_vals = 1;


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

q = [0,0,0];
q_dot = [];
v_dot = [];
v = [0,0,0];

angles = MadgwickAHRS()
def degrees(v):
	return [k*180/pi for k in v]

while not stop:
	time_now = time.time()
	accs = list(IMU.readAccData())
	gyros = [x for x in IMU.readGyroData()]

	deltime =  time_now - time_old;
	angles.update_imu(gyros,accs,0)

	q_dot = gyros;
	q = [ q_dot[k]*deltime + q[k] for k in range(len(q_dot))];


	rpy = angles.quaternion.to_euler_angles()
	# print("rpy: ", degrees(rpy))


	bot_matrix = R.from_euler('xyz',rpy).as_matrix();
	# print(bot_matrix)
	bT = Transformation()
	bT.rotate(bot_matrix)



	acc_in_W = [ 0, 0, 9.8];

	aW = Transformation();
	aW.locate(*acc_in_W);

	tA =  bT*aW;
	# print("tA")
	# print(tA)	
	corr = tA.T_matrix()[0:3,3]
	# print(corr)	

	v_dot = [accs[0]-corr[0],accs[1]-corr[1],accs[2] - corr[2]];
	v = [ v_dot[k]*deltime + v[k] for k in range(len(v_dot))];

	print("State")
	# print(q)
	print(v)


	icp_l = v[0]*85/8*0;
	icp_w = v[1]*73/8*0;

	# print(gyros)
	# if abs(gyros[0]) > abs(peak_x):
	# 	peak_x = gyros[0]
	# 	maxed_x = False;
	# 	fx_max = accs[0]
	# else:
	# 	maxed_x = True;
	# if abs(gyros[1]) > abs(peak_y):
	# 	peak_y = gyros[1]
	# 	maxed_y = False;
	# 	fy_max = accs[1]
	# else:
	# 	maxed_y = True;



	# stable_x = abs(gyros[0]) <= 0.01;
	# stable_y = abs(gyros[1]) <= 0.02;

	# peak_x = abs(gyros[0]) <= 0.02;
	# peak_y = abs(gyros[1]) < 0.02;

	if stable_x and stable_y:
		peak_x = 0;
		peak_y = 0;
		fx_max = 0;
		fy_max = 0;
		if time_old + time_delta < time_now:
			pct = traj_list[traj_index];
			_corr_traj = pct
			if traj_index == 1:
				_corr_traj = [pct[0] + icp_l,pct[1],pct[2],pct[3],pct[4] + icp_w,pct[5]]
			if traj_index == 4:
				_corr_traj = [pct[0],pct[1],pct[2] - icp_l,pct[3],pct[4],pct[5] - icp_w]

			mark_4.injest_ik_delay(_corr_traj,time_ms);
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
		step_len = 4;
		step_height = 25;

		# if gyros[1] > 0:
		# 	print("stepping back")
		# 	if gyros[2] < 0 :
		# 		print("left leg lift ")
		# 		left_leg_lift = step_height;
		# 		step_l = step_len;
		# 	else:
		# 		print("right leg lift ")
		# 		right_leg_lift = step_height;
		# 		step_r = -step_len;
		# else:
		# 	print("stepping forward")
		# 	if gyros[2] > 0 :
		# 		print("left leg lift ")
		# 		left_leg_lift = step_height;
		# 		step_l = -step_len;
		# 	else:
		# 		print("right leg lift ")
		# 		right_leg_lift = step_height;
		# 		step_r = step_len;
		if gyros[0] > 0 :
			print("left leg lift ")
			left_leg_lift = step_height;
			step_l = -step_len;
			step_r = step_len;
		else:
			print("right leg lift ")
			right_leg_lift = step_height;
			step_r = -step_len;
			step_l = step_len;

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