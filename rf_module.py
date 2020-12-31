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

"""
def write_data(inp_string):
    ser.write(inp_string.encode()) #need to turn input string into byte array for some reason
    
def read_data():
    try:
        while True: 
            data = ser.readline()
            print(data)
            time.sleep(0.2)
    except KeyboardInterrupt: 
        print("Press Ctrl-C to terminate while statement")
        pass
"""
