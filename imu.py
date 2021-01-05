"""This module serially reads the data from the IMU sensors""" 

import time
import board
import busio
import adafruit_lsm9ds1
from gpio import * 

# i2c = busio.I2C(board.SCL, board.SDA)
# sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# print(sensor.acceleration)
# print(sensor.magnetic)
# print(sensor.gyro)
# print(sensor.temperature)

i2c = busio.I2C(imu_scl, imu_sda)
imu = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

accelerometer = list(imu.acceleration)
magnetometer = list(imu.magnetic)
gyroscope = list(imu.gyro)
temp = imu.temperature

#this function is used to pretty print lists with three componenents (x, y, z) for coordinate labelling
def pretty_print(list):
#     data = [l
    #TODO: make imu data into a dictionary to include all sensors together? need to add temp? 
    coords = ["x: " + str(list[0]) + ", " + "y: " + str(list[1]) + ", " + "z: " + str(list[2])]
    print(coords)
    
def imu_print():
    pretty_print(accelerometer)
    pretty_print(magnetometer)
    pretty_print(gyroscope)
    #TODO: add csv write functionality for IMU noise


while True:
    imu_print()
    time.sleep(0.2)
# while True:
#     
# 
#     time.sleep(1)
