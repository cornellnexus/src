import threading
import numpy as np
import math

from engine.phase import Phase
from engine.ekf import LocalizationEKF
from constants.definitions import *

from engine.robot_logic.obstacle_avoidance import track_obstacle
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
            obstacle_avoidance = threading.Thread(target=track_obstacle, daemon=True)
            obstacle_avoidance.start()  # spawn thread to monitor obstacles
            robot.set_phase(Phase.TRAVERSE)
    else:
        robot.set_phase(Phase.TRAVERSE)