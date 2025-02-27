import math


import imu 
from step_bot import Bot
import time
from math import sin,cos,atan2,copysign,pi,sqrt
from scipy.spatial.transform import Rotation as R
from quaternion import Transformation
from mad_ahrs import MadgwickAHRS

from quaternion import Quaternion as qt
from quaternion import Haal


from bhram import Thing,Scene,bhram_vertex_shader,bhram_fragment_shader
from whiteroom import Room,make_goal_axis

from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *

SCENE_1 = Scene()
SCENE_1.fix_position([0,0,0])


_STATE_ = Thing(make_goal_axis,(2,10))
_STATE_ACC = Thing(make_goal_axis,(1,32))
SCENE_1.add_object(_STATE_)
SCENE_1.add_object(_STATE_ACC)



IMU = imu.Accelerometer()

mark_4 = Bot()
mark_4.home()
time.sleep(3)

sway = 0;
# sway = 3;
# 
# sh =  17;
# sh = 14
# sh = 17
sh = 0
# sh = 5
# sl = -4;
sl = 0
sb = 0;

tilt = 0;
lean = 0;


traj_list = [
	[ 0 + tilt + lean , 0, 0 + tilt  - lean, 0, sway, sway],
	[ -sh + sb, sh*2-sl, sb, 0, sway, sway],
	[0,0,0,0,0,0],
	[ 0 - tilt - lean , 0, 0 - tilt + lean , 0, -sway, -sway],
	[ -sb, 0, sh - sb, -sh*2+sl, -sway, -sway],
	[0,0,0,0,0,0],
]


N = len(traj_list)

traj_index = 0

# motor_update_rate = 8.5 #Hz
motor_update_rate = 12.5 #Hz

time_old = time.time();
time_delta = 1/motor_update_rate;
time_ms = int(time_delta*500);

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


fall_x = 0;
fall_y = 0;


def make_fall_vector(a,b):
	global fall_x,fall_y
	ang = atan2(fall_x,fall_y)*180/3.14159;
	mag = sqrt(fall_x*fall_x + fall_y*fall_y)*100;
	glPushMatrix()
	glColor3f(1.0, 1., 1.);
	glRotatef(ang,0,0,1);
	glTranslatef(b*a*mag/2,0,0);
	glScalef(a*mag,1,1);
	glutSolidCube(b)
	glPopMatrix()

_IMPULSE_ = Thing(make_fall_vector,(10,20))
SCENE_1.add_object(_IMPULSE_)

_ROBOT_ = Thing(mark_4.draw,())
SCENE_1.add_object(_ROBOT_)



SCENE_MAIN = Scene()

SCENE_MAIN.add_scene(SCENE_1,1,1,1)

R_viz = Room(SCENE_MAIN)
# angles = MadgwickAHRS()
def degrees(v):
	return [k*180/pi for k in v]

calibration = 1000;

while calibration:
	print("calibration")
	calibration -= 1;
	mark_4.readIMU()



mark_4.finish_calibration();
viability_x = 4;
viability_y = 3;
viability_z = 0.03;

while not stop:
	# global fall_x,fall_y
	time_now = time.time()
	# accs = list(IMU.readAccData())
	# gyros = [x for x in IMU.readGyroData()]

	deltime =  time_now - time_old;
	# angles.update_imu(gyros,accs,0)

	# q_dot = gyros;
	# q = [ q_dot[k]*deltime + q[k] for k in range(len(q_dot))];


	# rpy = angles.quaternion.to_euler_angles()
		# print("rpy: ", degrees(rpy))

	mark_4.readIMU();

	accs = mark_4.acc;
	gyros = mark_4.gyro;

	orie = mark_4.abs_orientation;
	bot_matrix = R.from_euler('xyz',orie).as_matrix();
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
	# print(gyros, end ="\t")	
	# print(accs)	

	v_dot = [accs[0]-corr[0],accs[1]-corr[1],accs[2] - corr[2]];
	v = [ v_dot[k]*deltime + v[k] for k in range(len(v_dot))];
	d_deg = degrees(orie)
	# print("State")
	print(d_deg)
	# print(v)


	icp_l = v[0]*25/8*0;
	icp_w = v[1]*13/8*0;

	fall_x = gyros[0];
	fall_y = gyros[1];
	# print(gyros)
	if abs(fall_x) > abs(peak_x):
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
		# fall_y = maxed_y;

	gm = R.from_euler('zyx',mark_4.orientation).as_quat();
	am = R.from_euler('xzy',[ 0 ,mark_4.beta ,mark_4.alpha]).as_quat();

	_STATE_.haal.rotation_Q = qt.from_value(gm);
	_STATE_ACC.haal.rotation_Q = qt.from_value(am);

	R_viz.update()
	# time.sleep(.1)


	
	deg_x = d_deg[0]
	deg_y = d_deg[1]
	vel_x = gyros[0]
	vel_y = gyros[0]
	deg_magnitude = math.sqrt(deg_x**2, deg_y**2)
	deg_dot_vel = (deg_x*vel_x + deg_y*vel_y) / deg_magnitude
	ellipsoid_radius = deg_dot_vel**2 / viability_z**2 +  deg_x**2 / viability_x**2 + deg_y**2 / viability_y**2

	if deg_dot_vel <= 0 or ellipsoid_radius < 1:
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
					# time.sleep(1)
		else:
			time.sleep(.001)

	else:
		if maxed_x and maxed_y:
			print("FALL START",end='\t');
			print(mark_4.frame,end='\t');
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
		step_len = 1;
		step_height = 5;
		lean_amount = 10;

		if gyros[1] > 0:
			leanl  = lean_amount;
		else:
			leanl =  lean_amount;
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
		# if gyros[0] > 0 :
		# 	print("left leg lift ")
		# 	left_leg_lift = step_height;
		# 	step_l = -step_len;
		# 	step_r = step_len;
		# else:
		# 	print("right leg lift ")
		# 	right_leg_lift = step_height;
		# 	step_r = -step_len;
		# 	step_l = step_len;

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


		# mark_4.injest_ik_delay([step_l - left_leg_lift + leanl,2*left_leg_lift,- step_r + right_leg_lift - leanl,-2*right_leg_lift,kick_l,kick_r],50);
		# stop = True
		# time.sleep(.1)
		# # mark_4.injest_ik_delay([0 ,0,-0 - 0,0,kick_l,kick_r],200);
		# # time.sleep(1)


		# mark_4.injest_ik_delay([0,0,0,0,0,0],200);
		# time.sleep(.5)			
		# if marignal_toq > 0:
		# 	# Right step 
		# 	mark_4.injest_ik_delay([roll_y,0,-roll_y,0,kick_l,kick_r],20);
		# else:
			# Left step 
	# print(gyros)