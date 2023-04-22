from engine.robot_logic.robot import Robot
from engine.robot_logic.set_up import execute_setup
from engine.phase import Phase
from engine.robot_state import Robot_State
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
        robot_state = Robot_State(xpos=0, ypos=0, heading=math.pi / 4, epsilon=0.2, max_velocity=0.5, radius=0.2, phase = Phase.SETUP)
        self.robot = Robot(robot_state)
        self.gps_serial = serial.Serial('/dev/ttyACM0', 19200, timeout=5) 
        self.imu_i2c = busio.I2C(board.SCL, board.SDA)
        self.gps = GPS(self.gps_serial, False) 
        self.imu = IMU(self.imu_i2c, False) 
        self.robot_radio_serial = serial.Serial('/dev/ttyS0', 57600) #robot radio device
        self.robot_radio_session = RadioModule(self.robot_radio_serial) 
        self.motor_controller = MotorController(wheel_r = 0, vm_load1 = 1, vm_load2 = 1, L = 0, R = 0, is_sim = self.robot.robot_state.is_sim)
    
    def run(self): 
        execute_setup(self.robot_radio_session, self.gps, self.imu, self.motor_controller)

class BaseStationSetupPhaseTest: 
    """
    Test Script for Setup Phase that the radio module is setup properly on the base station
    """
    def __init__(self): 
        self.basestation_radio_session = RadioModule(None)

    def run(self): 
        pass
        #TODO: change this into a base station execute_setup function
        # execute_setup(self.base_session, self.gps, self.imu, self.pid_motor)

#TODO:  	
#check phase at the end
#check imu data that comes out 
#check gps data that comes out 
#check radio connection 
#check motor controller data? 
#maybe want to create some logger for these tests when run on terminal
