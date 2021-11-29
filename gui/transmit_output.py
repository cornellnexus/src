import os
import sys

import serial

#ser = serial.Serial("/dev/cu.usbserial-017543DC", 57600)

def send_lines():
    robot_command_file = open((get_path('csv')[-1] + '/robot_command.csv'), "r+")  # open csv file of robot data
    lines = robot_command_file.readlines()
    for line in lines:
        line_as_bytes = str.encode(line)
#        ser.write(line_as_bytes)
        print(line_as_bytes.decode())
    robot_command_file.seek(0)
    robot_command_file.truncate()


def get_path(folder):

    cwd = os.getcwd()
    sys.path.append(cwd + "/" + folder)
    return sys.path
