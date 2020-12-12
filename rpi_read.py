import os, sys
import serial
import time
import pynmea2
import csv


ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 5)

def parseGPS(str):
    if str.find('GGA') > 0:
        msg = pynmea2.parse(str)
        return (msg.longitude, msg.latitude)

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
