# -----------------------------------------------------------------------------
# Team: Nexus at Cornell
# Author: Alan Hsiao
# Date: 12_20_2021
# Script: motor_control.py
# Description: Test the servos
# -----------------------------------------------------------------------------

import RPi.GPIO as GPIO
import time

# set up PWM pins and frequency
PWMR = 12
PWML = 13
freq = 50

# set up PWM with frequency
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWMR, GPIO.OUT)
GPIO.setup(PWML, GPIO.OUT)
pR = GPIO.PWM(PWMR, freq)
pL = GPIO.PWM(PWML, freq)

# =====================
#      TEST MOTORS
# =====================
# 0 for CCW, 1 for CW
# 0 for left, 1 for right
# duty cycle from 0-100
def motorCommand(robotSide, cw, dc):
    if cw == 1:
        output = 7.5 - dc / 100
    else:
        output = 7.5 + dc / 100
    if robotSide == 0:
        pL.ChangeDutyCycle(output)
    else:
        pR.ChangeDutyCycle(output)


# =====================
#      	 DC CALC
# =====================
def dutyCalc(cw, dc):
    if cw == 1:
        output = 7.5 - dc / 100
    else:
        output = 7.5 + dc / 100


# =====================
#      TEST ROBOT
# =====================
# 0 F, 1 B, 2 L, 3 R
def robotCommand(direction):
    if direction == 0:
        pL.ChangeDutyCycle(8.5)
        pR.ChangeDutyCycle(6.5)
        time.sleep(3.5)
    elif direction == 1:
        pL.ChangeDutyCycle(6.5)
        pR.ChangeDutyCycle(8.5)
        time.sleep(3.5)
    elif direction == 2:
        pL.ChangeDutyCycle(8.5)
        pR.ChangeDutyCycle(8)
        time.sleep(4)
    elif direction == 3:
        pL.ChangeDutyCycle(6.5)
        pR.ChangeDutyCycle(6.5)
        time.sleep(4)


pR.start(0)
pL.start(0)

# robotSide = input("Enter 0 for Left, 1 for Right: ")
# cw = input("Enter 0 for CCW, 1 for CW: ")
# dc = input("Enter duty cycle (0-100): ")
# motorCommand(robotSide, cw, dc)

val = input("Enter Direction: ")
robotCommand(val)

pR.stop()
pL.stop()

GPIO.cleanup()
