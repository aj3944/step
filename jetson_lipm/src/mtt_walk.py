


import imu 
from step_bot import Bot
import time
from math import sin,cos,atan2,copysign
import threading

IMU = imu.Accelerometer()

mark_4 = Bot()
mark_4.home()
time.sleep(3)

sway = 2.5;



traj_list = [
	[ 0, 0, 0, 0, sway, sway],
	[ 0, 0, 0, 0, -sway, -sway],
]


N = len(traj_list)

traj_index = 0

motor_update_rate = 1 #Hz
imu_update_rate = 10 #Hz
control_update_rate = 1 #Hz

# time_old = time.time();
motor_time_delta = 1/motor_update_rate;
imu_time_delta = 1/imu_update_rate;
control_time_delta = 1/control_update_rate;

max_vals = 20;


stop = False;



falling = False;
fall_left = False;
# fall_right = False;
stable_x = True
stable_y = True
accs = []
gyros = []

def check_stability(imu):
	global stable_x,stable_y,fall_front,fall_left,stop,accs,gyros,imu_time_delta
	time_old = time.time()
	while not stop:
		time_now = time.time()
		if time_old + imu_time_delta < time_now:
			accs = list(imu.readAccData())
			gyros = [x for x in imu.readGyroData()]
			stable_x = abs(gyros[0]) < 0.02;
			stable_y = abs(gyros[1]) < 0.02;
			# print(gyros)
			fall_front = copysign(1,gyros[0])
			fall_right = copysign(1,gyros[1])
			time_old = time.time()
			print(stable_x,stable_y,fall_front,fall_right)

t1 = threading.Thread(target=check_stability, args=(IMU,))

exec_trajec = []

future_traj = []
future_traj_timing = []



def make_trajectory():
	global stable_x,stable_y,fall_front,fall_left,traj_list,traj_index
	global  future_traj,future_traj_timing,stop,accs,gyros,control_time_delta
	time_old = time.time()
	while not stop:
		time_now = time.time()
		if time_old + control_time_delta < time_now:
			if stable_x and stable_y:
				if len(future_traj) > 10:
					continue
				future_traj.append(traj_list[traj_index])
				future_traj_timing.append(500)
				traj_index += 1;
				traj_index %= N;
			else:
				print("INSTABILITY!!!")
				future_traj = []
				future_traj_timing = []
				kick_l = 0;
				kick_r = 0;
				roll_y = 0;
				step_l = 0;
				step_r = 0;
				kick_l = 20*abs(gyros[0]);
				kick_r = -20*abs(gyros[0]);
				future_traj.append([roll_y,0,-roll_y,0,kick_l,kick_r])
				future_traj_timing.append(50)		
				future_traj.append([0,0,0,0,0,0])
				future_traj_timing.append(200)
				stable_x = True
				stable_y = True		
			time_old = time.time()

t2 = threading.Thread(target=make_trajectory, args=())

def execute_trajectory():
	global stop,motor_time_delta,future_traj_timing,future_traj,mark_4
	time_old = time.time()
	while not stop:
		time_now = time.time()
		if len(future_traj) <= 0:
			print("no traj")
			continue
		# else:
		# 	print(len(future_traj))
		if time_old + motor_time_delta < time_now:
			next_traj = future_traj[0]
			next_timing = future_traj_timing[0]
			print(next_traj)
			print(next_timing)
			mark_4.injest_ik_delay(next_traj,next_timing);
			future_traj = future_traj[1:]
			future_traj_timing = future_traj_timing[1:]
			# traj_index += 1
			# motor_time_delta = 2*next_timing/1000;
			time_old = time.time()


t3 = threading.Thread(target=execute_trajectory, args=())


t1.start();
t2.start();

execute_trajectory();


# t3.start();

# t1.join();
# t2.join();
# t3.join();

