import ast
from datetime import datetime
import json
import serial


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
        self.imu_dict = {}
        self.created = datetime.now().strftime("data/%d-%m-%Y_%H:%M:%S")
        self.write_data = write

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
            imu_file = open("data/" + self.created + ".txt", 'w+')
            imu_file.write(json.dumps(self.imu_dict) + "\n")
            imu_file.close()

