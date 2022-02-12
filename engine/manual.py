# TODO: documentation
import os
import sys
from electrical.commands import *
from engine.mission import ControlMode


class Manual():
    # TODO: move functions from gui and robot here for proper categorization

    def __init__(self, control_mode):
        self.control_mode = ControlMode(control_mode)

    # commented part of the code is reading serially. not commented part is testing via csv files
    def get_control_mode(self):
        robot_command_file = open((self.get_path('csv')[-1] + '/control_mode_test.csv'), "r+")
        # byte_data = ser.readline()
        # data = byte_data.decode()
        last_line = robot_command_file.readlines()[-1]
        last_line = last_line.strip()
        control_mode = last_line[last_line.index(".") + 1:len(last_line)]
        if control_mode == 'Manual':
            self.control_mode = ControlMode(Manual)
        # problem right now is that gui sends only "manual" or "autonomous" so robot_data is where we contain the
        # control modes. However, robot data is a csv file, which is something on the gui side that the robot
        # will not have access to.
        elif control_mode == 'Autonomous':
            self.control_mode = ControlMode()

    def execute_manual(self):
        robot_data_file = open((self.get_path('csv')[-1] + '/robot_command.csv'), "r+")  # open csv file of robot data
        # instead of rereading serialized data (which I don't know works), save the data in a field of Manual and pull from there.
        # byte_data = ser.readline()
        # data = byte_data.decode()
        datas = robot_data_file.readlines()
        for data in datas:
            data = data.strip()
            if data == 'left':
                turn_left()
            elif data == 'right':
                turn_right()
            elif data == 'forward':
                go_forward()
            elif data == 'backward':
                reverse()
        robot_data_file.seek(0)
        robot_data_file.truncate()

    def get_path(self, folder): #only useful for testing. Remove when reading file serially.

        cwd = os.getcwd()
        sys.path.append(cwd + "/" + folder)
        return sys.path
