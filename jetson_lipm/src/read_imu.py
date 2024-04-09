
import time
import random

import imu 


LIMIT_TIME = 20  # s
DATA_FILENAME = "data.txt"


IMU = imu.Accelerometer()



def gen_data(filename, limit_time):
    start_time = time.time()
    elapsed_time = time.time() - start_time
    curr_time = time.time()
    with open(filename, "w") as f:
        while curr_time <   start_time + limit_time:
            curr_time = time.time()
            print(start_time)
            accs = list(IMU.readAccData())
            gyros = list(IMU.readGyroData())
            total = accs + gyros
            print(curr_time)
            f.write(f"{time.time():12.2f}:{total}\n")  # produces 64 bytes
            f.flush()
            elapsed = time.time() - start_time
            time.sleep(0.01)
            

gen_data(DATA_FILENAME, LIMIT_TIME)
