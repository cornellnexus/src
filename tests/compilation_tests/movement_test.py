# file for testing gps data
import RPi.GPIO as GPIO
from engine.robot import Robot
from engine.robot_state import Robot_State
import time

from electrical.motor_controller import BasicMotorController, MotorController

# comment out one of the tests to run
############################### Testing GPS ##################################
# ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 5)
#
# def print_gps(str):
#     if str.find('GGA') > 0:
#         msg = pynmea2.parse(str)
#         print (msg.longitude, msg.latitude)
#
# while True:
#     line = str(ser.readline())
#     decoded_line = line[2:-5]
#     if len(decoded_line) == 0:
#         print("Time out! exit. \n")
#         sys.exit()
#     print_gps(decoded_line)

############################### Testing GPIO #################################
robot_state = Robot_State(
    xpos=0, ypos=0, heading=0, epsilon=0, max_velocity=0, radius=1
)
robot = Robot(robot_state)
motor_controller = BasicMotorController(robot.robot_state.is_sim)

stopTime = time.time() + 10
while time.time() < stopTime:
    motor_controller.go_forward()

motor_controller.stop()

GPIO.cleanup()
