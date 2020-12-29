""" This module receives the serially transmitted data from the rf module"""

import time 
import serial 


# Replace "replace" with Computer port
ser = serial.Serial(port = "replace", 9600)

while True: 
  data = ser.readline()
  print(data)
  time.sleep(0.2)

#TODO: 3dr transmission - raspberry pi ports