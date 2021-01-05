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
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)


while True:
    acceleration = sensor.acceleration
    magnetometer = sensor.magnetic
    gyroscope = sensor.gyro
    temp = sensor.temperature

    print("Acceleration (m/s^2):" + str(acceleration)) 
    print("Magnetometer (gauss):" + str(magnetometer))
    print("Gyroscope (degrees/sec)" + str(gyroscope))
    print("Temperature (C)" + str(temp))
    print("--------------------")
    time.sleep(0.2)

    time.sleep(1)
