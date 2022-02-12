import threading
import time
import board
import busio
from adafruit_lsm9ds1 import LSM9DS1_I2C

# random change message

# --------------------------------READ ME BEFORE LOOKING AT CODE-------------------------------
"""
Most of this code came from this website: https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration/calibration-with-raspberry-pi-using-blinka.
I changed things slightly here and there to so that it might work with our imu.
From some research, for the magnetometer there are 2 types of offset values that we need to keep
in mind: hard-offset and soft-offset. Subtract each x, y, z hard-iron offset from the respective x, y, z
magnetometer readings. Do similar thing with soft-iron offset values.

Similar process done with gyroscope data (except using zero-rate offsets)

site I used to help understand things --> https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration 
"""
# ---------------------------------------------------------------------------------------------

i2c = busio.I2C(board.SCL, board.SDA)

accelerometer = magnetometer = gyro_accel = LSM9DS1_I2C(i2c)

while time.time() < time.time() + 60:
    acc_x, acc_y, acc_z = accelerometer.acceleration
    print(
        "Accelerometer: X: {0:8.5f}, Y:{1:8.5f}, Z:{2:8.5f} rad/s".format(acc_x, acc_y, acc_z))
    gyro_x, gyro_y, gyro_z = gyro_accel.gyro
    print(
        "Gyroscope: X: {0:8.5f}, Y:{1:8.5f}, Z:{2:8.5f} rad/s".format(gyro_x, gyro_y, gyro_z))
    mag_x, mag_y, mag_z = magnetometer.magnetic
    print("Magnetometer: X: {0:8.5f}, Y:{1:8.5f}, Z:{2:8.5f} uT".format(
        mag_x, mag_y, mag_z))
