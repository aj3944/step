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


# This code is taken from the Adafruit LSMD9S1 library.
# Credit to Tony DiCola for writing.
# The original code is available at:
# https://github.com/adafruit/Adafruit_CircuitPython_LSM9DS1/blob/master/adafruit_lsm9ds1.py

# from micropython import  

# pylint: disable=bad-whitespace
_LSM9DS1_ADDRESS_ACCELGYRO       =  (0x6B)
_LSM9DS1_ADDRESS_MAG             =  (0x1E)
_LSM9DS1_XG_ID                   =  (0b01101000)
_LSM9DS1_MAG_ID                  =  (0b00111101)
_LSM9DS1_ACCEL_MG_LSB_2G         = 0.061
_LSM9DS1_ACCEL_MG_LSB_4G         = 0.122
_LSM9DS1_ACCEL_MG_LSB_8G         = 0.244
_LSM9DS1_ACCEL_MG_LSB_16G        = 0.732
_LSM9DS1_MAG_MGAUSS_4GAUSS       = 0.14
_LSM9DS1_MAG_MGAUSS_8GAUSS       = 0.29
_LSM9DS1_MAG_MGAUSS_12GAUSS      = 0.43
_LSM9DS1_MAG_MGAUSS_16GAUSS      = 0.58
_LSM9DS1_GYRO_DPS_DIGIT_245DPS   = 0.00875
_LSM9DS1_GYRO_DPS_DIGIT_500DPS   = 0.01750
_LSM9DS1_GYRO_DPS_DIGIT_2000DPS  = 0.07000
_LSM9DS1_TEMP_LSB_DEGREE_CELSIUS = 8
_LSM9DS1_REGISTER_WHO_AM_I_XG    =  (0x0F)
_LSM9DS1_REGISTER_CTRL_REG1_G    =  (0x10)
_LSM9DS1_REGISTER_CTRL_REG2_G    =  (0x11)
_LSM9DS1_REGISTER_CTRL_REG3_G    =  (0x12)
_LSM9DS1_REGISTER_TEMP_OUT_L     =  (0x15)
_LSM9DS1_REGISTER_TEMP_OUT_H     =  (0x16)
_LSM9DS1_REGISTER_STATUS_REG     =  (0x17)
_LSM9DS1_REGISTER_OUT_X_L_G      =  (0x18)
_LSM9DS1_REGISTER_OUT_X_H_G      =  (0x19)
_LSM9DS1_REGISTER_OUT_Y_L_G      =  (0x1A)
_LSM9DS1_REGISTER_OUT_Y_H_G      =  (0x1B)
_LSM9DS1_REGISTER_OUT_Z_L_G      =  (0x1C)
_LSM9DS1_REGISTER_OUT_Z_H_G      =  (0x1D)
_LSM9DS1_REGISTER_CTRL_REG4      =  (0x1E)
_LSM9DS1_REGISTER_CTRL_REG5_XL   =  (0x1F)
_LSM9DS1_REGISTER_CTRL_REG6_XL   =  (0x20)
_LSM9DS1_REGISTER_CTRL_REG7_XL   =  (0x21)
_LSM9DS1_REGISTER_CTRL_REG8      =  (0x22)
_LSM9DS1_REGISTER_CTRL_REG9      =  (0x23)
_LSM9DS1_REGISTER_CTRL_REG10     =  (0x24)
_LSM9DS1_REGISTER_OUT_X_L_XL     =  (0x28)
_LSM9DS1_REGISTER_OUT_X_H_XL     =  (0x29)
_LSM9DS1_REGISTER_OUT_Y_L_XL     =  (0x2A)
_LSM9DS1_REGISTER_OUT_Y_H_XL     =  (0x2B)
_LSM9DS1_REGISTER_OUT_Z_L_XL     =  (0x2C)
_LSM9DS1_REGISTER_OUT_Z_H_XL     =  (0x2D)
_LSM9DS1_REGISTER_WHO_AM_I_M     =  (0x0F)
_LSM9DS1_REGISTER_CTRL_REG1_M    =  (0x20)
_LSM9DS1_REGISTER_CTRL_REG2_M    =  (0x21)
_LSM9DS1_REGISTER_CTRL_REG3_M    =  (0x22)
_LSM9DS1_REGISTER_CTRL_REG4_M    =  (0x23)
_LSM9DS1_REGISTER_CTRL_REG5_M    =  (0x24)
_LSM9DS1_REGISTER_STATUS_REG_M   =  (0x27)
_LSM9DS1_REGISTER_OUT_X_L_M      =  (0x28)
_LSM9DS1_REGISTER_OUT_X_H_M      =  (0x29)
_LSM9DS1_REGISTER_OUT_Y_L_M      =  (0x2A)
_LSM9DS1_REGISTER_OUT_Y_H_M      =  (0x2B)
_LSM9DS1_REGISTER_OUT_Z_L_M      =  (0x2C)
_LSM9DS1_REGISTER_OUT_Z_H_M      =  (0x2D)
_LSM9DS1_REGISTER_CFG_M          =  (0x30)
_LSM9DS1_REGISTER_INT_SRC_M      =  (0x31)
_MAGTYPE                         = True
_XGTYPE                          = False
_SENSORS_GRAVITY_STANDARD        = 9.80665

# User facing  ants/module globals.
ACCELRANGE_2G                = (0b00 << 3)
ACCELRANGE_16G               = (0b01 << 3)
ACCELRANGE_4G                = (0b10 << 3)
ACCELRANGE_8G                = (0b11 << 3)
MAGGAIN_4GAUSS               = (0b00 << 5)  # +/- 4 gauss
MAGGAIN_8GAUSS               = (0b01 << 5)  # +/- 8 gauss
MAGGAIN_12GAUSS              = (0b10 << 5)  # +/- 12 gauss
MAGGAIN_16GAUSS              = (0b11 << 5)  # +/- 16 gauss
GYROSCALE_245DPS             = (0b00 << 3)  # +/- 245 degrees/s rotation
GYROSCALE_500DPS             = (0b01 << 3)  # +/- 500 degrees/s rotation
GYROSCALE_2000DPS            = (0b11 << 3)  # +/- 2000 degrees/s rotation
# pylint: enable=bad-whitespace
