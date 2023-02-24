""" Module that includes functions for GPS sensor  """

import serial
import time
import pynmea2
import csv
from engine.is_raspberrypi import is_raspberrypi
if is_raspberrypi():
    from ublox_gps import UbloxGps


class GPS:
    def __init__(self, init_serial, is_sim):
        self.is_sim = is_sim
        if not is_sim: 
            self.ser = init_serial
            self.gps = UbloxGps(self.ser)


    """ get_gps: returns the coordinate (long, lat)"""

    def get_gps(self):
        """
        gps_line = str(gps.stream_nmea())
        while gps_line.find("GGA") < 0:
            gps_line = str(gps.stream_nmea())
        coord = self.parse_gps(gps_line)
        if coord is not None:
            return (coord)
        """
        if not self.is_sim: 
            geo = self.gps.geo_coords()
            return {"long": geo.lon, "lat": geo.lat}

    """ parse_gps[gps_line]: helper that takes in serial gps data and returns
        a coord in the form (longitude, latitude).
        precondition: gps_line must be a string of pynmea format.
    """

    def parse_gps(self, gps_line):
        if not self.is_sim: 
            msg = pynmea2.parse(gps_line)
            data = {"long": msg.longitude, "lat": msg.latitude}
            return (data)

    """ setup: function that checks the gps data for longitude and latitude 
        is not 0 when robot is in startup phase. 
        returns True when GPS is setup properly, False if not"""
    def setup(self): 
        gps_data = []
        count = 0 
        if not self.is_sim: 
            while (len(gps_data) < 25): 
                count += 1 
                data = self.get_gps()
                if (data.get("long") != 0 and data.get("lat") != 0): 
                    gps_data.append(data)
                if (count > 200): 
                    return False
            return True 