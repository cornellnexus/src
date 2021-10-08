import ast
from datetime import datetime
import json
import serial


class SensorModule:
    def __init__(self, write):
        self.port = serial.Serial(port="/dev/tty.usbserial-017543DC", baudrate=57600, timeout=0)
        self.port.flushInput()
        self.imu_dict = {}
        self.created = datetime.now().strftime("%d/%m/%Y_%H:%M:%S")
        self.write_data = write

    def update_imu_data(self):
        if self.port.in_waiting > 90:
            line = self.port.readline().decode("utf-8")
            if line[-1] == "\n" and line[0] == "{":
                self.imu_dict = ast.literal_eval(line.rstrip("\n"))

        if self.write_data:
            imu_file = open(self.created + ".txt")
            imu_file.write(json.dumps(self.imu_dict))
            imu_file.close()

