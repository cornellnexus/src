# TODO: documentation
import os
import sys
from electrical.commands import *
from engine.mission import ControlMode


class Manual():

    def __init__(self, control_mode):
        self.control_mode = ControlMode(control_mode)

    def get_control_mode(self):
        robot_command_file = open((self.get_path('csv')[-1] + '/control_mode_test.csv'), "r+")
        # byte_data = ser.readline()
        # data = byte_data.decode()
        last_line = robot_command_file.readlines()[-1]
        last_line = last_line.strip()
        control_mode = last_line[last_line.index(".") + 1:len(last_line)]
        if control_mode == 'Manual':
            return ControlMode.MANUAL
        elif control_mode == 'Autonomous':
            return self.control_mode

    def execute_manual(self):
        robot_data_file = open((self.get_path('csv')[-1] + '/robot_command.csv'), "r+")  # open csv file of robot data
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

    def get_path(self, folder):

        cwd = os.getcwd()
        sys.path.append(cwd + "/" + folder)
        return sys.path
