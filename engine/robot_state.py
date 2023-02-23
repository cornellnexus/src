import numpy as np
from engine.phase import Phase
from electrical.imu import IMU
from electrical.gps import GPS
from engine.is_raspberrypi import is_raspberrypi
import math


class Robot_State:
    """
        Variables:
            x_pos: the x position of the robot, where (0,0) is the bottom left corner of the grid with which
                the Robot's related Mission was initialized
            y_pos: the y position of the robot
            heading: the theta of the robot in radians, where North on the grid is equal to 0.
            epsilon: dictates the turning radius of the robot. Lower epsilon results in tighter turning radius.
            max_v: the maximum velocity of the robot
            radius: the radius of the robot
            width: width of the robot in cm
            front_ultrasonic: the ultrasonic at the front of the robot, used for detecting obstacles
            lf_ultrasonic: the ultrasonic at the front of the left side of the robot, used for boundary following
            lb_ultrasonic: the ultrasonic at the back of the left side of the robot, used for boundary following
            rf_ultrasonic: the ultrasonic at the front of the right side of the robot, used for boundary following
            rb_ultrasonic: the ultrasonic at the back of the right side of the robot, used for boundary following
            is_sim: False if the physical robot is being used, True otherwise
            is_store: False if csv data should not be stored, True otherwise
            position_kp: the proportional factor of the position PID
            position_ki: the integral factor of the position PID
            position_kd: the derivative factor of the position PID
            position_noise: the flat amount of noise added to the robot's phase on each localization step
            heading_kp: the proportional factor of the heading PID
            heading_ki: the integral factor of the heading PID
            heading_kd: the derivative factor of the heading PID
            heading_noise: ?
            init_phase: the phase which the robot begins at
            time_step: the amount of time that passes between each feedback loop cycle, should only be used if is_sim
                is True
            move_dist: the distance in meters that the robot moves per time dt
            turn_angle: the angle in radians that the robot turns per time dt regardless of time step
            plastic_weight: the weight of the trash the robot has collected
            init_threshold (Double): Radius from initial position that will detect robot is back in initial position
            goal_threshold (Double): Threshold from goal that will be detected as reaching goal in obstacle avoidance
            noise_margin (Double): Margin from init_threshold that the robot has to leave before detecting robot has
                left initial position
    """

    def __init__(self, x_pos, y_pos, heading, epsilon, max_velocity, radius):
        # User defined parameters
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.heading = heading
        self.epsilon = epsilon
        self.max_velocity = max_velocity
        self.radius = radius

        # Defaults
        self.is_sim = not is_raspberrypi()
        self.is_store = False
        self.is_sim = True
        self.position_kp = 1
        self.position_ki = 0
        self.position_kd = 0
        self.position_noise = 0
        self.heading_kp = 1
        self.heading_ki = 0
        self.heading_kd = 0
        self.heading_noise = 0
        self.phase = Phase.TRAVERSE
        self.time_step = 1
        self.move_dist = 0.5
        self.turn_angle = 3
        self.plastic_weight = 0

        self.state = np.array([[self.x_pos], [self.y_pos], [self.heading]])
        self.truthpose = np.transpose(
            np.array([[self.x_pos], [self.y_pos], [self.heading]]))

        self.battery = 100
        # self.imu = IMU(init_i2c=None, is_sim=self.is_sim)
        self.acceleration = [0, 0, 0]
        self.magnetic_field = [0, 0, 0]
        self.gyro_rotation = [0, 0, 0]
        self.linear_v = 0
        self.angular_v = 0

        self.init_gps = (0, 0)
        self.gps_data = (0, 0)
        self.imu_data = None  # will be filled by execute_setup
        self.ekf_var = None
        self.mc = None
        self.using_odom = False

        self.width = 700
        self.avoid_obstacle = False  # boolean that determines if we should avoid obstacles
        self.front_ultrasonic = None
        self.lf_ultrasonic = None
        self.lb_ultrasonic = None
        self.rf_ultrasonic = None
        self.rb_ultrasonic = None
        self.dist_to_goal = 0
        self.prev_phase = self.phase
        self.goal_location = (0, 0)
        self.max_sensor_range = 600
        # TODO: replace this with how far offset the sensor is to the front of the robot
        self.front_sensor_offset = 0
        self.sensor_measuring_angle = 75
        self.width_margin = 1  # TODO: replace this with actual margin
        self.threshold_distance = ((self.width + self.width_margin) / 2) / math.cos(
            math.radians((180 - self.sensor_measuring_angle) / 2))
        self.detect_obstacle_range = min(self.threshold_distance,
                                         self.max_sensor_range)  # set ultrasonic detection range
        self.init_threshold = 1
        self.goal_threshold = 1
        self.noise_margin = 1
        if not self.is_sim:
            from electrical.motor_controller import MotorController
            from electrical.radio_module import RadioModule
            import serial
            import busio
            import board
            self.motor_controller = MotorController(
                self, wheel_radius=0, vm_load1=1, vm_load2=1, L=0, R=0)
            self.robot_radio_session = RadioModule(
                serial.Serial('/dev/ttyS0', 57600))
            self.gps = GPS(serial.Serial('/dev/ttyACM0', 19200, timeout=5))
            self.imu = IMU(busio.I2C(board.SCL, board.SDA))
            self.start_coor = GPS.get_gps()

    # def __init__(self, x_pos, y_pos, heading, epsilon, max_v, radius, is_sim=True, is_store=False, width=700, front_ultrasonic=None,
    #              lf_ultrasonic=None, lb_ultrasonic=None, rf_ultrasonic=None, rb_ultrasonic=None,
    #              position_kp=1, position_ki=0, position_kd=0, position_noise=0, heading_kp=1, heading_ki=0,
    #              heading_kd=0, heading_noise=0, init_phase=1, time_step=1, move_dist=.5, turn_angle=3,
    #              plastic_weight=0, use_ekf=False, init_gps=(0, 0), gps_data=(0, 0), imu_data=None, ekf_var=None,
    #              gps=None, imu=None, init_threshold=1, goal_threshold=1, noise_margin=1, motor_controller=None):
    #     """
    #     Arguments:
    #         x_pos: the x position of the robot, where (0,0) is the bottom left corner of the grid with which
    #             the Robot's related Mission was initialized
    #         y_pos: the y position of the robot
    #         heading: the theta of the robot in radians, where North on the grid is equal to 0.
    #         epsilon: dictates the turning radius of the robot. Lower epsilon results in tighter turning radius.
    #         max_v: the maximum velocity of the robot
    #         radius: the radius of the robot
    #         width: width of the robot in cm
    #         front_ultrasonic: the ultrasonic at the front of the robot, used for detecting obstacles
    #         lf_ultrasonic: the ultrasonic at the front of the left side of the robot, used for boundary following
    #         lb_ultrasonic: the ultrasonic at the back of the left side of the robot, used for boundary following
    #         rf_ultrasonic: the ultrasonic at the front of the right side of the robot, used for boundary following
    #         rb_ultrasonic: the ultrasonic at the back of the right side of the robot, used for boundary following
    #         is_sim: False if the physical robot is being used, True otherwise
    #         is_store: False if csv data should not be stored, True otherwise
    #         position_kp: the proportional factor of the position PID
    #         position_ki: the integral factor of the position PID
    #         position_kd: the derivative factor of the position PID
    #         position_noise: the flat amount of noise added to the robot's phase on each localization step
    #         heading_kp: the proportional factor of the heading PID
    #         heading_ki: the integral factor of the heading PID
    #         heading_kd: the derivative factor of the heading PID
    #         heading_noise: ?
    #         init_phase: the phase which the robot begins at
    #         time_step: the amount of time that passes between each feedback loop cycle, should only be used if is_sim
    #             is True
    #         move_dist: the distance in meters that the robot moves per time dt
    #         turn_angle: the angle in radians that the robot turns per time dt regardless of time step
    #         plastic_weight: the weight of the trash the robot has collected
    #         init_threshold (Double): Radius from initial position that will detect robot is back in initial position
    #         goal_threshold (Double): Threshold from goal that will be detected as reaching goal in obstacle avoidance
    #         noise_margin (Double): Margin from init_threshold that the robot has to leave before detecting robot has
    #             left initial position
    #     """
    #     self.state = np.array([[x_pos], [y_pos], [heading]])
    #     self.truthpose = np.transpose(np.array([[x_pos], [y_pos], [heading]]))
    #     self.is_sim = not is_raspberrypi()
    #     self.is_store = is_store
    #     self.phase = Phase(init_phase)
    #     self.epsilon = epsilon
    #     self.max_velocity = max_v
    #     self.radius = radius
    #     self.time_step = time_step
    #     self.position_kp = position_kp
    #     self.position_ki = position_ki
    #     self.position_kd = position_kd
    #     self.position_noise = position_noise
    #     self.heading_kp = heading_kp
    #     self.heading_ki = heading_ki
    #     self.heading_kd = heading_kd
    #     self.heading_noise = heading_noise
    #     self.move_dist = move_dist
    #     # dividing by time_step ignores the effect of time_step on absolute
    #     self.turn_angle = turn_angle / time_step
    #     self.plastic_weight = plastic_weight
    #     self.battery = 100  # TEMPORARY
    #     self.acceleration = [0, 0, 0]  # TEMPORARY
    #     self.magnetic_field = [0, 0, 0]  # TEMPORARY
    #     self.gyro_rotation = [0, 0, 0]  # TEMPORARY
    #     self.init_gps = init_gps
    #     self.gps_data = gps_data
    #     self.imu_data = imu_data  # will be filled by execute_setup
    #     self.ekf_var = ekf_var
    #     self.gps = gps
    #     self.imu = imu
    #     self.mc = motor_controller
    #     self.linear_v = 0
    #     self.angular_v = 0
    #     self.width = width
    #     self.avoid_obstacle = False  # boolean that determines if we should avoid obstacles
    #     self.front_ultrasonic = front_ultrasonic
    #     self.lf_ultrasonic = lf_ultrasonic
    #     self.lb_ultrasonic = lb_ultrasonic
    #     self.rf_ultrasonic = rf_ultrasonic
    #     self.rb_ultrasonic = rb_ultrasonic
    #     self.dist_to_goal = 0
    #     self.prev_phase = self.phase
    #     self.goal_location = (0, 0)
    #     self.max_sensor_range = 600
    #     self.front_sensor_offset = 0  # TODO: replace this with how far offset the sensor is to the front of the robot
    #     self.sensor_measuring_angle = 75
    #     self.width_margin = 1  # TODO: replace this with actual margin
    #     self.threshold_distance = ((self.width + self.width_margin) / 2) / math.cos(
    #         math.radians((180 - self.sensor_measuring_angle) / 2))
    #     self.detect_obstacle_range = min(self.threshold_distance,
    #                                      self.max_sensor_range)  # set ultrasonic detection range
    #     self.init_threshold = init_threshold
    #     self.goal_threshold = goal_threshold
    #     self.noise_margin = noise_margin
    #     if not self.is_sim:
    #         self.motor_controller = MotorController(self, wheel_radius = 0, vm_load1 = 1, vm_load2 = 1, L = 0, R = 0)
    #         self.robot_radio_session = RadioModule(serial.Serial('/dev/ttyS0', 57600))
    #         self.gps = GPS(serial.Serial('/dev/ttyACM0', 19200, timeout=5))
    #         self.imu = IMU(busio.I2C(board.SCL, board.SDA))

    # def rpiToGui():
    #     pass
