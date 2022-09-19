from engine.robot import Robot, Phase
from electrical.radio_module import RadioModule
from electrical.motor_controller import MotorController
from electrical.imu import IMU
from electrical.gps import GPS

import serial
import busio
import board
import time
import math


if True: #change to True when running code on robot
    import RPi.GPIO as GPIO

class RobotSetupPhaseTest:
    """
    Test Script for Setup Phase that checks GPS, IMU, Radio Module, and Motors
    are setup properly on the robot.
    """
    def __init__(self): 
        self.robot = Robot(0, 0, math.pi / 4, epsilon=0.2, max_v=0.5, radius=0.2, init_phase=Phase.SETUP)
        self.gps_serial = serial.Serial('/dev/ttyACM0', 19200, timeout=5) 
        self.imu_i2c = busio.I2C(board.SCL, board.SDA)
        self.gps = GPS(self.gps_serial) 
        self.imu = IMU(self.imu_i2c) 
        self.robot_radio_serial = serial.Serial('/dev/ttyS0', 57600) #robot radio device
        self.robot_radio_session = RadioModule(self.robot_radio_serial) 
        self.motor_controller = MotorController(robot = self.robot, wheel_r = 0, vm_load1 = 1, vm_load2 = 1, L = 0, R = 0)
    
    def run(self): 
        self.robot.execute_setup(self.robot_radio_session, self.gps, self.imu, self.motor_controller)

class BaseStationSetupPhaseTest: 
    """
    Test Script for Setup Phase that the radio module is setup properly on the base station
    """
    def __init__(self): 
        self.basestation_radio_session = RadioModule(None)

    def run(self): 
        pass 


    def run(self): 
        pass
        #TODO: change this into a base station execute_setup function
        # self.robot.execute_setup(self.base_session, self.gps, self.imu, self.pid_motor)

#TODO:  	
#check phase at the end
#check imu data that comes out 
#check gps data that comes out 
#check radio connection 
#check motor controller data? 
#maybe want to create some logger for these tests when run on terminal
