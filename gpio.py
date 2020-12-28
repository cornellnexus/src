""" This module contains all the pinout establishments and setup functions """

import RPi.GPIO as GPIO
import time

#Motor Driver
in1 = 27
in2 = 29
in3 = 31
in4 = 33
enA = 35
enB = 37

#IMU
imu_sda = 3
imu_scl = 5

#Sonar
sonar_trig = 16
sonar_echo = 18 #double check this order

#RF Module 

def setup():
    GPIO.setmode(GPIO.BOARD)
    #Motor Driver
    GPIO.setup([in1, in2, in3, in4], GPIO.OUT, initial=GPIO.LOW) #In1, In2, In3, In4
    GPIO.setup([enA, enB], GPIO.OUT) #EnA, EnB

    e1 = GPIO.PWM(enA, 600)    # create object D2A for PWM on port 25 at 1KHz
    e2 = GPIO.PWM(enB, 600)

    e1.start(100)
    e2.start(100)

    #IMU
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

    # RF 

    # GPS
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 5)

    # Sonar
    # GPIO.setup(sonar_trig, GPIO.OUT)
    # GPIO.setup(sonar_echo, GPIO.IN)


