""" Module that includes functions for GPS sensor  """

import os, sys
import serial
import time
import pynmea2
import csv

def parse_gps(str):
  if str.find('GGA') > 0: 
    msg = pynmea2.parse(str)
    return(msg.longitude, msg.latitude)


def update_step(): 
  line = ser.readline()
  if len(line) == 0: 
    print("Time out! exit. \n")
    sys.exit()
  coord = parse_gps(line)
  with open('gps_noise.csv', 'w') as gps_file: 
    gps_file.write(str(coord) + '\n')


update_step()
