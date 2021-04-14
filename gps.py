""" Module that includes functions for GPS sensor  """

import serial
import time
import pynmea2
import csv
from gpio import *

ser = serial.Serial('/dev/ttyACM0', 19200, timeout=5)

""" parse_gps[gps_line]: takes in serial gps data and returns
    a coordinate in the form (longitude, latitude).
    precondition: gps_line must be a string of pynmea format.
"""


def parse_gps(gps_line):
    msg = pynmea2.parse(gps_line)
    data = (msg.longitude, msg.latitude)
    return(data)


""" update_step: returns the tuple (long, lat).
    note: if the string 'GGA' is not found in the serial data, update_step is called again."""


def update_step():
    gps_line = str(ser.readline())
    decoded_line = gps_line[2:-5]
    flag = decoded_line.find('GGA')
    if flag < 0:
        print("No GGA")
        update_step()
    else:
        coord = parse_gps(decoded_line)
        print("coord: ", coord)
        return(coord)
