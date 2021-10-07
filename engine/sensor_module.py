import serial
import ast


class SensorModule:
    def __init__(self):
        self.port = serial.Serial(port="/dev/tty.usbserial-017543DC", baudrate=57600, timeout=0)
        self.port.flushInput()
        self.imu_dict = {}

    def update_imu_data(self):
        if self.port.in_waiting > 90:
            line = self.port.readline().decode("utf-8")
            # ----------------------Get dict from string------------------------

            if line[-1] == "\n" and line[0] == "{":
                self.imu_dict = ast.literal_eval(line.rstrip("\n"))
