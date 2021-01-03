""" Module contains commands to physically move the robot. """

# TODO: 
# turnLeft(): change the number of seconds the robot turns to get 90º
# turnRight(): change the number of seconds the robot turns to get 90º 

import RPi.GPIO as GPIO
import time
from gpio import *

# stops the robot 
def stop():
    GPIO.output([in1, in2, in3, in4],GPIO.LOW)

# moves the robot forward 
def go_forward():
    GPIO.output([in1, in4],GPIO.LOW)
    GPIO.output([in2, in3], GPIO.HIGH)

# reverses the robot 
def reverse():
    GPIO.output([in2, in3], GPIO.LOW)
    GPIO.output([in1, in4],GPIO.HIGH)

# turns the robot left for 1 second 
def turn_left():
    GPIO.output([in2, in4],GPIO.LOW)
    GPIO.output([in1, in3], GPIO.HIGH)
    time.sleep(1)

# turns the robot right for 1 second 
def turn_right():
    GPIO.output([in1, in3],GPIO.LOW)
    GPIO.output([in2, in4], GPIO.HIGH)
    time.sleep(1)

# pauses the robot for 0.2 seconds
def pause():
    GPIO.output([in1, in2, in3, in4], GPIO.LOW) #In1, In2, In3, In4
    time.sleep(0.2)

# testing the functions to run under 10 seconds 
stopTime = time.time() + 10
# while time.time() < stopTime:
#     turn_left()
#     pause()
#     go_forward()
#     stop()
# e1.stop()
# e2.stop()

# cleans up all the ports used for motor driver 
GPIO.cleanup()

