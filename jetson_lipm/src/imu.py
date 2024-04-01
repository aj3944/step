# The MIT License (MIT)
#
# Copyright (c) 2017 Tony DiCola for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN


# Much of this code is taken from the Adafruit LSMD9S1 library.
# modifications are primarily to use the smbus library instead of the
# Adafruit CircuitPython package

# Credit to Tony DiCola for writing.
# The original code is available at:
# https://github.com/adafruit/Adafruit_CircuitPython_LSM9DS1/blob/master/adafruit_lsm9ds1.py

import smbus2 as smbus
import time
import struct

import LSM9DS1 as LS

def twos_comp(val, bits):
    if val & (1 << (bits - 1)) != 0:
        return val - (1 << bits)
    return val

class Accelerometer:
    # Driver for the 9-axis Accelerometer, Magnetometer and Gyroscope
    # Uses SMBus as the interfacing tool over i2c

    # Init the smbus connection = defaults to bus 1 but that can be changed
    # by changing the argument
    bus = smbus.SMBus(1)

    # This slightly simplifies addressing the two devices on the LSM9DS1 as the
    # values are used often.
    addMag = (LS._LSM9DS1_ADDRESS_MAG)
    addAccelGyro = (LS._LSM9DS1_ADDRESS_ACCELGYRO)

    def __init__(self):
        self.detectDevice()
        self.softResetAll()
        self.setDefaults()
        self.enableAllContinuous()
        self._accel_mg_lsb = LS._LSM9DS1_ACCEL_MG_LSB_2G
        self._mag_mgauss_lsb = LS._LSM9DS1_MAG_MGAUSS_4GAUSS
        self._gyro_dps_digit = LS._LSM9DS1_GYRO_DPS_DIGIT_245DPS

    # Sends a reset signal to each device

    def softResetAll(self):
        self.writeToDevice(self.addAccelGyro, LS._LSM9DS1_REGISTER_CTRL_REG8, 0x05)
        self.writeToDevice(self.addMag, LS._LSM9DS1_REGISTER_CTRL_REG2_M, 0x0C)
        time.sleep(0.5)

    # Ensures that the device is connected, raising an error if not and
    # printing a message if it responds as expected. 

    def detectDevice(self):
        if self.readFromDevice(self.addAccelGyro, LS._LSM9DS1_REGISTER_WHO_AM_I_XG) != LS._LSM9DS1_XG_ID or \
           self.readFromDevice(self.addMag, LS._LSM9DS1_REGISTER_WHO_AM_I_M) != LS._LSM9DS1_MAG_ID:
            raise RuntimeError('Could not find the device. Please check the wiring.')
        print('Device found')

    # Sets up the devices for continuous operation

    def enableAllContinuous(self):
        self.writeToDevice(self.addAccelGyro, LS._LSM9DS1_REGISTER_CTRL_REG1_G, 0xC0)
        self.writeToDevice(self.addAccelGyro, LS._LSM9DS1_REGISTER_CTRL_REG5_XL, 0x38)
        self.writeToDevice(self.addAccelGyro, LS._LSM9DS1_REGISTER_CTRL_REG6_XL, 0xC0)
        self.writeToDevice(self.addMag, LS._LSM9DS1_REGISTER_CTRL_REG3_M, 0x00)

    # Gives default range values to the three devices. TODO add specific setter
    # methods for each
    
    def setDefaults(self):
        # Set Accelerometer range to 2G
        accelRange = 0xc0
        accelRange = (accelRange & ~(0b00011000)) & 0xFF
        accelRange |= (0b00 << 3)
        self.writeToDevice(self.addAccelGyro, LS._LSM9DS1_REGISTER_CTRL_REG6_XL, accelRange)

        self._accel_mg_lsb = LS._LSM9DS1_ACCEL_MG_LSB_2G

        # Set Magnetometer gain to 4Gauss
        magGain = self.readFromDevice(self.addMag, LS._LSM9DS1_REGISTER_CTRL_REG2_M)
        magGain = (magGain & ~(0b01100000)) & 0xFF
        magGain |= LS.MAGGAIN_4GAUSS
        self.writeToDevice(self.addMag, LS._LSM9DS1_REGISTER_CTRL_REG2_M, magGain)

        self._mag_mgauss_lsb = LS._LSM9DS1_MAG_MGAUSS_4GAUSS
        
        # Set Gyro Scale to 245DPS
        gyroScale = self.readFromDevice(self.addAccelGyro, LS._LSM9DS1_REGISTER_CTRL_REG1_G)
        gyroScale = (gyroScale & ~(0b00011000)) & 0xFF
        gyroScale |= LS.GYROSCALE_245DPS
        self.writeToDevice(self.addAccelGyro, LS._LSM9DS1_REGISTER_CTRL_REG1_G, gyroScale)

        self._gyro_dps_digit = LS._LSM9DS1_GYRO_DPS_DIGIT_245DPS


    # Reads the raw data then outputs as a 3-tuple of gauss values

    def readMagData(self):
        raw = self.getXYZ(self.addMag, LS._LSM9DS1_REGISTER_OUT_X_L_M)

        return map(lambda x: round(x *(self._mag_mgauss_lsb / 1000.0), 2), raw)

    def readAccData(self):

        raw = self.getXYZ(self.addAccelGyro, LS._LSM9DS1_REGISTER_OUT_X_L_XL)

        return map(lambda x: round(x * (self._accel_mg_lsb/ 1000.0) * LS._SENSORS_GRAVITY_STANDARD, 2),
                   raw)
  
    def readGyroData(self):
        raw = self.getXYZ(self.addAccelGyro, LS._LSM9DS1_REGISTER_OUT_X_L_G)

        return map(lambda x: round(x * (self._gyro_dps_digit / 1000.0), 2), raw)

    # General method to write data to the device

    def writeToDevice(self, deviceAddress, internalAddress, data):
        self.bus.write_byte_data(deviceAddress, internalAddress, data)

    # General method to read data from the device

    def readFromDevice(self, deviceAddress, internalAddress):
        return self.bus.read_byte_data(deviceAddress, internalAddress)

    # Generic to get x, y, z data from devices

    def getXYZ(self, device, startByte):
        data = self.bus.read_i2c_block_data(device, startByte, 6)

        x = twos_comp(int(data[0] | (data[1] << 8)), 16)
        y = twos_comp(int(data[2] | (data[3] << 8)), 16)
        z = twos_comp(int(data[4] | (data[5] << 8)), 16)

        return (x, y, z)
