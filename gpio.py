""" This module contains all the pinout establishments and setup functions.
    Note: all written connections are Raspberry pi 3b+ Broadcom numbers"""

import RPi.GPIO as GPIO
import time

# import time
# import board
# import busio
# import adafruit_lsm9ds1


#Motor Driver
in1 = 5
in2 = 6
in3 = 19
in4 = 26
enA = 13
enB = 12

#IMU
imu_sda = 2
imu_scl = 3

#Sonar
sonar_trig = 23
sonar_echo = 24 #double check this order

#RF Module 
rf_tx = 14
rf_rx = 15


# def setup():
#     GPIO.setmode(GPIO.BCM)
#     #Motor Driver
#     GPIO.setup([in1, in2, in3, in4], GPIO.OUT, initial=GPIO.LOW) #In1, In2, In3, In4
#     GPIO.setup([enA, enB], GPIO.OUT) #EnA, EnB
# 
#     e1 = GPIO.PWM(enA, 600)    # create object D2A for PWM on port 25 at 1KHz
#     e2 = GPIO.PWM(enB, 600)
# 
#     e1.start(100)
#     e2.start(100)

#     #IMU
#     i2c = busio.I2C(imu_scl, imu_sda)
#     sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
# 
#     # RF 
# 
#     # GPS
#     ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 5)
# 
#     # Sonar
#     # GPIO.setup(sonar_trig, GPIO.OUT)
#     # GPIO.setup(sonar_echo, GPIO.IN)

# setup()
# GPIO.cleanup()



