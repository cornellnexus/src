#file for testing gps data
import RPi.GPIO as GPIO
from gpio import *
import pynmea2
import serial

#comment out one of the tests to run
############################### Testing GPS ##################################
ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 5)

def print_gps(str):
    if str.find('GGA') > 0:
        msg = pynmea2.parse(str)
        print (msg.longitude, msg.latitude)
        
while True:
    line = ser.readline()
    print_gps(line)
  
############################### Testing GPIO #################################
setup()
while True:
    goForward()

