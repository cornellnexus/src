""" Module contains commands to physically move the robot. """

import RPi.GPIO as GPIO
import time
from gpio import *

GPIO.setmode(GPIO.BCM)
# Motor Driver
GPIO.setup([in1, in2, in3, in4], GPIO.OUT, initial=GPIO.LOW)  # In1, In2, In3, In4
GPIO.setup([enA, enB], GPIO.OUT)  # EnA, EnB

e1 = GPIO.PWM(enA, 600)  # create object D2A for PWM on port 25 at 1KHz
e2 = GPIO.PWM(enB, 600)

e1.start(100)
e2.start(100)


# stops the robot 
def stop():
    # GPIO.output([in1, in2, in3, in4],GPIO.LOW)
    print('stop')


# moves the robot forward
def go_forward():
    # GPIO.output([in1, in4],GPIO.LOW)
    # GPIO.output([in2, in3], GPIO.HIGH)
    print('go_forward')
    time.sleep(0.1)


# reverses the robot
def reverse():
    # GPIO.output([in2, in3], GPIO.LOW)
    # GPIO.output([in1, in4],GPIO.HIGH)
    print('reverse')


# turns the robot left for 1 second
def turn_left():
    # GPIO.output([in2, in4],GPIO.LOW)
    # GPIO.output([in1, in3], GPIO.HIGH)
    print('turn_left')
    time.sleep(1)


# turns the robot right for 1 second
def turn_right():
    # GPIO.output([in1, in3],GPIO.LOW)
    # GPIO.output([in2, in4], GPIO.HIGH)
    print('turn_right')
    time.sleep(1)


# pauses the robot for 0.2 seconds
def pause():
    # GPIO.output([in1, in2, in3, in4], GPIO.LOW) #In1, In2, In3, In4
    print('pause')
    time.sleep(0.2)

# testing the functions to run under 10 seconds 
# stopTime = time.time() + 5
# while time.time() < stopTime:
#     go_forward()
# # # 
# # stop()
# # 

# # cleans up all the ports used for motor driver 
# GPIO.cleanup()