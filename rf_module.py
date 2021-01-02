""" This module receives the serially transmitted data from the rf module"""

import time 
import serial 
from gpio import * 

# Ground Station 
ser_gnd = serial.Serial(port = "computer port", 9600, timeout = 0)

def gnd_receive(): 
  while ser_gnd.inWaiting():  
      received_data_gnd = ser_gnd.readline()
      print(received_data)
      time.sleep(0.2)

def gnd_transmit(data): 
  ser_gnd.write(data)

# RPi Station
ser_rpi = serial.Serial(port = "rpi port", 9600, timeout = 0)

def rpi_receive(): 
  while ser_rpi.inWaiting():  
      received_data_rpi = ser_rpi.readline()
      print(received_data)
      time.sleep(0.2)

def rpi_transmit(data):
  ser_rpi.write(data)

