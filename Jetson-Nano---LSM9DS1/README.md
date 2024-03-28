# Jetson-Nano---LSM9DS1
This repo allows a user to use the LSM9DS1 all-in-one accelerometer / gyro / magnetometer on the Jetson Nano, using i2c.

Much of this code is taken from the Adafruit LSMD9S1 library.
modifications are primarily to use the smbus library instead of the
Adafruit CircuitPython package

Credit to Tony DiCola for writing.
The original code is available at:
https://github.com/adafruit/Adafruit_CircuitPython_LSM9DS1/blob/master/adafruit_lsm9ds1.py


This is tested on Python 2.7 using the Adafruit LSM9DS1 module - though should work with python 3.x and any module using 
the ST Microelectronics LSM9DS1.

Datasheet for the module is at https://www.st.com/en/mems-and-sensors/lsm9ds1.html


It comprises two files: 

LSM9DS1.py
Contains the register addresses and other constants necessary to operate the device.
This is separated out to allow later modification to generalise the code to other devices should it be necessary. 

accelerometer.py
Contains the class Accelerometer. 


Usage:

Connect the device to i2c channel 1:

SCL - Pin 5

SDA - Pin 3

(This is hardcoded but may later be modified to be changable)

Copy both files into the active directory and import accelerometer.py

Make sure that you have downloaded and installed smbus:

```
pip install smbus
```

On instantiation the devices are reset and default ranges are added, and each device is set to continuous mode. 


To create an instance of Accelerometer:

```
device = accelerometer.Accelerometer()
```

Receive the accelerometer data as a 3-tuple formatted (x, y, z) in m/s:

```
device.readAccData()
```

Receive the gyroscope data as a 3-tuple formatted (x, y, z) in d/s:

```
device.readGyroData()
```

Receive the magnetometer data as a 3-tuple formatted (x, y, z). Not sure what unit it is though!:

```
device.readMagData()
```

The range of the devices are hardcoded as follows:

Accelerometer:    2G

Gyroscope:        245DPS

Magnetometer:     2 Gauss

Code to modify these will be added at a later date.

