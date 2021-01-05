#file for testing gps data
import RPi.GPIO as GPIO
from gpio import *
import pynmea2
import serial

import os, sys
import time
import csv

from commands import *


#comment out one of the tests to run
############################### Testing GPS ##################################
# ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 5)

def print_gps(str):
    if str.find('GGA') > 0:
        msg = pynmea2.parse(str)
        print (msg.longitude, msg.latitude)
        
# while True:
#     line = str(ser.readline())
#     decoded_line = line[2:-5]
#     if len(decoded_line) == 0: 
#         print("Time out! exit. \n")
#         sys.exit()
#     print_gps(decoded_line)
  
############################### Testing GPIO #################################
# setup()
stopTime = time.time() + 10
while time.time() < stopTime:
#     go_forward()
    stop()

# stop()

