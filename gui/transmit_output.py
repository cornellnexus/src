import os
import sys

import serial

# ser = serial.Serial("/dev/cu.usbserial-017543DC", 57600)
# TODO: move functions here from robot and gui for proper categorization. Step 2 would be making sure data is not corrupt
def send_command_lines():
    # robot_command_file = open((get_path('csv')[-1] + '/robot_command.csv'), "r+")  # open csv file of robot data
    # lines = robot_command_file.readlines()
    # for line in lines:
    #     line_as_bytes = str.encode(line)
    #     ser.write(line_as_bytes)
    #     print(line_as_bytes.decode())
    # robot_command_file.seek(0)
    # robot_command_file.truncate()
    # execute_commands()
    pass


def get_path(folder):

    cwd = os.getcwd()
    sys.path.append(cwd + "/" + folder)
    return sys.path

def send_control_mode():
    # robot_control_file = open((get_path('csv')[-1] + '/control_mode_test.csv'), "r+")  # open csv file of robot data
    # lines = robot_control_file.readlines()
    # for line in lines:
    #     line_as_bytes = str.encode(line)
    #     ser.write(line_as_bytes)
    #     print(line_as_bytes.decode())
    # robot_command_file.seek(0)
    # robot_command_file.truncate()
    # execute_commands()
    pass
