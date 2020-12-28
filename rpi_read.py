#This module reads the data from the GPS and IMU sensors 

import os, sys
import serial
import time
import pynmea2
import csv
from imu import * 

# ---------------------------- GPS module -------------------------------------

# parseGPS(str) takes an input [str] of the NMEA 0183 format and returns 
# the (longitude, latitude) of the GPS data 
def parseGPS(str):
    if str.find('GGA') > 0:
        msg = pynmea2.parse(str)
        return (msg.longitude, msg.latitude)

# update_step() returns and writes the updated GPS coordinates to a csv file. 
# Returns "Time out! exit" if GPS data is not being transmitted properly. 
def update_step():
    line = ser.readline()
    if len(line) == 0:
        print("Time out! exit.\n")
        sys.exit()
    coord = parseGPS(line)

    with open('estimated_coords.csv', mode='w') as coords_file:
        cf_writer = csv.writer(coords_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        cf_writer.writerow([coord[0], coord[1]])
    return(parseGPS(line))

# ---------------------------- IMU module -------------------------------------
#TODO: Place IMU function calls here. 
