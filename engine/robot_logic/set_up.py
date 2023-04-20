import threading
import math
import time
import numpy as np

from engine.phase import Phase
from engine.ekf import LocalizationEKF
from constants.definitions import *

from engine.is_raspberrypi import is_raspberrypi
if is_raspberrypi():
    # Electrical library imports
    from electrical.motor_controller import MotorController
    import electrical.gps as GPS 
    import electrical.imu as IMU 
    import electrical.radio_module as RadioModule
    import serial
    import board
    import busio

def track_obstacle(robot):
    """ Continuously checks if there's an obstacle in the way.
        Will be executing on a separate, asynchronous thread.
        Precondition:
            Front ultrasonic sensor is mounted in the center front/x position of the robot (width/2)
    """
    # assuming front sensor is mounted in the center x position (width/2)
    # initialize ultrasonics
    front_ultrasonic = None
    counter = 0  # added for testing
    while True:
        if robot.robot_state.is_sim:
            ultrasonic_value_file = open(ROOT_DIR + '/tests/functionality_tests/csv/ultrasonic_values.csv',
                                            "r")  # added for testing
            content = ultrasonic_value_file.readlines()
            ultrasonic_value_file.close()
            try:
                line = content[counter]
                counter += 1
                curr_ultrasonic_value = float((''.join(line.rstrip('\n')).strip('()').split(', '))[0])
            except IndexError:
                print("no more sensor data")
                break
        else:
            from electrical.ultrasonic_sensor import Ultrasonic
            front_ultrasonic = Ultrasonic(0)
            curr_ultrasonic_value = front_ultrasonic.distance()
            if curr_ultrasonic_value < robot.robot_state.front_sensor_offset:
                robot.set_phase(Phase.FAULT)
                return None
        if robot.robot_state.is_roomba_traversal:  # roomba mode
            robot.robot_state.is_roomba_obstacle = True
        elif (robot.robot_state.phase in [Phase.TRAVERSE, Phase.RETURN, Phase.DOCKING, Phase.AVOID_OBSTACLE]):
            if curr_ultrasonic_value < robot.robot_state.detect_obstacle_range:
                # Note: didn't check whether we can reach goal before contacting obstacle because obstacle
                # detection does not detect angle, so obstacle could be calculated to be falsely farther away than
                # the goal. Not optimal because in cases, robot will execute boundary following when it can reach
                # goal
                robot.set_phase(Phase.AVOID_OBSTACLE)
                if robot.robot_state.is_sim:
                    with open(ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv', 'a') as fd:
                        fd.write("Avoid" + '\n')
            else:
                if robot.robot_state.is_sim:
                    with open(ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv', 'a') as fd:
                        fd.write("Not Avoid" + '\n')
        if curr_ultrasonic_value < 0:
            # value should not go below 0; sensor is broken
            robot.set_phase(Phase.FAULT)
            if robot.robot_state.is_sim:
                with open(ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv', 'a') as fd:
                    fd.write("Fault" + '\n')
        if not robot.robot_state.is_sim:
            time.sleep(10)  # don't hog the cpu

def execute_setup(robot):
    if not robot.is_sim:
        robot.robot_state.motor_controller = MotorController(wheel_radius = 0, vm_load1 = 1, vm_load2 = 1, L = 0, R = 0, is_sim = robot.is_sim)
        robot.robot_state.robot_radio_session = RadioModule(serial.Serial('/dev/ttyS0', 57600)) 
        robot.robot_state.gps = GPS(serial.Serial('/dev/ttyACM0', 19200, timeout=5), is_sim = robot.is_sim) 
        robot.robot_state.imu = IMU(init_i2c = busio.I2C(board.SCL, board.SDA), is_sim = robot.is_sim) 
    
        gps_setup = robot.robot_state.gps.setup()
        imu_setup = robot.robot_state.imu.setup()
        robot.robot_state.radio_session.setup_robot()
        robot.robot_state.motor_controller.setup(robot.robot_state.is_sim)

        robot.robot_state.init_gps = (robot.robot_state.gps.get_gps()["long"], robot.robot_state.gps.get_gps()["lat"])
        robot.robot_state.imu_data = robot.robot_state.imu.get_gps()
        x_init, y_init = (0, 0)
        heading_init = math.degrees(math.atan2(
            robot.robot_state.imu_data["mag"]["y"], robot.robot_state.imu_data["mag"]["x"]))

        # mu is meters from start position (bottom left position facing up)
        mu = np.array([[x_init], [y_init], [heading_init]])

        # confidence of mu, set it to high initially b/c not confident, algo brings it down
        sigma = np.array([[10, 0, 0], [0, 10, 0], [0, 0, 10]])
        robot.robot_state.ekf = LocalizationEKF(mu, sigma)

        if (robot.robot_state.radio_session.connected and gps_setup and imu_setup):
            obstacle_avoidance = threading.Thread(target=track_obstacle, args=robot, daemon=True)
            obstacle_avoidance.start()  # spawn thread to monitor obstacles
            robot.set_phase(Phase.TRAVERSE)
    else:
        robot.set_phase(Phase.TRAVERSE)
    
