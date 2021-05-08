""" Module that includes functions for GPS sensor  """

import serial
import time
import pynmea2
import csv
from gpio import *

class GPS:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 19200, timeout=5)

    """ parse_gps[gps_line]: takes in serial gps data and returns
        a coord in the form (longitude, latitude).
        precondition: gps_line must be a string of pynmea format.
    """
    def parse_gps(self, gps_line):
        msg = pynmea2.parse(gps_line)
        data = (msg.longitude, msg.latitude)
        return(data)

    """ update_step: returns the tuple (long, lat)."""
    def update_step(self):
        gps_line = str(self.ser.readline())
        decoded_line = gps_line[2:-5]
        while decoded_line.find('GGA') < 0:
            gps_line = str(self.ser.readline())
            decoded_line = gps_line[2:-5]
        coord = self.parse_gps(decoded_line)
        if coord is not None:
            return(coord)
