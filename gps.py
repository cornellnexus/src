""" Module that includes functions for GPS sensor  """

import os, sys
import serial
import time
import pynmea2
import csv
from gpio import *

ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 5)


def parse_gps(str):
    if str.find('GGA') > 0: 
        msg = pynmea2.parse(str)
        data = (msg.longitude, msg.latitude)
        return(data)
#     print("GGA not found")



def update_step():
    line = str(ser.readline())
    decoded_line = line[2:-5]
#     print(decoded_line)
    if len(decoded_line) == 0: 
        print("Time out! exit. \n")
        sys.exit()
    coord = parse_gps(decoded_line)
    if isinstance(coord, tuple):
        csv_data.append(coord)
        print(csv_data)


csv_data = []
num_coords = 5

# update_step()
def write_to_csv(csv_data):
    while len(csv_data) < num_coords:
        update_step()
    with open('gps_noise.csv','w') as gps_file:
        for coord in csv_data:
            gps_file.write(str(coord) + '\n')
            
write_to_csv(csv_data)
