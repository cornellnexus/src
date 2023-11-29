import ast
from datetime import datetime
import json
import serial
import numpy as np
import math

from engine.kinematics import get_vincenty_x, get_vincenty_y
from electrical import breakbeam


class SensorModule:
    def __init__(self, write):
        """
        An instance of the SensorModule class represents the set of sensors used by the robot.

        Arguments:
            write: True if IMU data should be written to a file, False if not. The file is named after the current
            datetime.
        """
        self.port = serial.Serial(port="/dev/tty.usbserial-017543DC", baudrate=57600, timeout=0)
        self.port.flushInput()
        self.imu_dict = {"mag": {"x": 0, "y": 0}}
        self.created = datetime.now().strftime("%d-%m-%Y_%H_%M_%S")
        self.write_data = write
        self.gps_dict = {"lon": 0, "lat": 0}

        # HALF1 = 17
        # HALF2 = 22
        # FULL1 = 23
        # FULL2 = 24
        pins = [ 17, 22, 23, 24 ]
        self.breakbeam = breakbeam(pins)

    def update_imu_data(self):
        """
        Retrieves 9-degree IMU data from the Raspberry Pi.
        Writes IMU data to a file if self.write is True.
        """
        if self.port.in_waiting > 90:
            line = self.port.readline().decode("utf-8")
            if line[-1] == "\n" and line[0] == "{":
                self.imu_dict = ast.literal_eval(line.rstrip("\n"))

        if self.write_data:
            imu_file = open("csv/IMU_" + self.created + ".txt", 'w+')
            imu_file.write(json.dumps(self.imu_dict) + "\n")
            imu_file.close()

    def update_gps_data(self):
        """
        Retrieves GPS data from the Raspberry Pi.
        Writes GPS data to a file if self.write is True.
        """
        if self.port.in_waiting > 0:
            line = self.port.readline().decode("utf-8")
            self.gps_dict = ast.literal_eval(line.rstrip("\n"))

        if self.write_data:
            gps_file = open("csv/GPS_" + self.created + ".txt", 'a')
            gps_file.write(json.dumps(self.gps_dict) + "\n")
            gps_file.close()

        print(self.gps_dict)

    def get_measurement(self, origin):
        """
        Args:
            origin - (lat, long) of the origin of the global frame.
        Returns:
            measurement - a 3x1 matrix of measurements. x_position, y_position, theta
        """
        measurement = np.zeros((3, 1))

        x_measurement = get_vincenty_x(origin, (self.gps_dict["lat"], self.gps_dict["lon"]))
        y_measurement = get_vincenty_y(origin, (self.gps_dict["lat"], self.gps_dict["lon"]))
        theta_measurement = math.degrees(math.atan2(self.imu_dict["mag"]["y"], self.imu_dict["mag"]["x"]))

        measurement[0] = x_measurement
        measurement[1] = y_measurement
        measurement[2] = theta_measurement

        return measurement
