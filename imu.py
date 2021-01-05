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

accelerometer = imu.acceleration
magnetometer = imu.magnetic
gyroscope = imu.gyro
temp = imu.temperature

def map_iterator(map):
    for data in map:
        print(str(data) + " ")
    print(' ')

def print_imu_data():
    map_iterator(accelerometer)
#     print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accelerometer))
#     print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(magnetometer))
#     print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyroscope))
#     print('Temperature: {0:0.3f}C'.format(temperature))

print_imu_data()

# while True:
#     
# 
#     time.sleep(1)
