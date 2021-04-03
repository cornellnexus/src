""" Module that includes functions for GPS sensor  """

import os, sys
import serial
import time
import pynmea2
import csv
from gpio import *

ser = serial.Serial('/dev/ttyACM0', 19200, timeout = 5)
csv_data = []
collect_time = time.time() + 60
# CODE does not work atm. error. 
def gps():
    gps_line = ser.readline()
    if len(gps_line)==0:
        print("exiting. timeout")
        sys.exit()
    while gps_line.find('GGA')<0:
        print("re-reading")
        gps_line = ser.readline()
    gps_data = pynmea2.parse(gps_line)
    my_lat = float(gps_data.latitude)
    my_lon = float(gps_data.longitude)
    print((my_lon, my_lat))
    print("----------------------")
    return(my_lon, my_lat)

while(True):
    gps()
    delay(2)


# def parse_gps(serial_line):
#     msg = pynmea2.parse(serial_line)
#     print(msg)
#     data = (msg.longitude, msg.latitude)
#     return(data)

# def update_step():
#     line = str(ser.readline())
#     decoded_line = line[2:-5]
#     if len(decoded_line) == 0: 
#         print("Time out! exit. \n")
#         sys.exit()
#     while line.find('GGA') < 0: 
#         print("no GGA -- gps_line: ",line)
#         line = str(ser.readline())
#     coord = parse_gps(decoded_line)
#     if isinstance(coord, tuple): 
#         print("tuple coord: ", coord)
#         return(coord)
    # if isinstance(coord, tuple):
    #     print('----------------GOT COORD!!!!!----------------')
    #     print('is tuple: ' + str(coord))
    #     return(coord)
#     print('not tuple: '+str(coord))
# 
# while(True):
#     update_step()
#     delay(2)