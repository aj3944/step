


import imu 
from step_bot import Bot
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation





IMU = imu.Accelerometer()

# fig = plt.figure()
# ax1 = fig.add_subplot(1,1,1)

fig, imu_values = plt.subplots(2, 1, figsize=(10,8))



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

motor_update_rate = 1 #Hz

time_old = time.time();
time_delta = 1/motor_update_rate;

xs = []
ys = [
	[],[],[],[],[],[]
]

max_vals = 20
def animate(i):
	global time_old,time_delta,xs,ys,traj_index,traj_list,max_vals
	time_now = time.time()
	if time_old + time_delta < time_now:
		traj_index += 1;
		traj_index %= N;
		mark_4.injest_ik_delay(traj_list[traj_index]);
		time_old = time.time()
	accs = list(IMU.readAccData())
	gyros = [x/100 for x in IMU.readGyroData()]
	ys[0].append(accs[0]);
	ys[1].append(accs[1]);
	ys[2].append(accs[2]);
	ys[3].append(gyros[0]);
	ys[4].append(gyros[1]);
	ys[5].append(gyros[2]);
	xs.append(i);
	for i in range(6):
		ys[i] = ys[i][-max_vals:];
	xs = xs[-max_vals:];
	print(gyros[0]);
	# imu_values[0].clear();
	imu_values[1].clear();
	# imu_values[0].plot(xs, ys[0],label="accl x")
	# imu_values[0].plot(xs, ys[1],label="accl y")
	# imu_values[0].plot(xs, ys[2],label="accl z")
	# plt.legend(loc="upper left")
	imu_values[1].plot(xs, ys[3],label="gyro x")
	# imu_values[1].plot(xs, ys[5],label="gyro y")
	# imu_values[1].plot(xs, ys[4],label="gyro z")
	plt.legend(loc="upper left")


ani = animation.FuncAnimation(fig, animate, interval=100)
plt.show()
