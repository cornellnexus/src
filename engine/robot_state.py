import math
import numpy as np
from engine.phase import Phase

# Sensor dependent packages
from engine.is_raspberrypi import is_raspberrypi
if is_raspberrypi():
    from electrical.motor_controller import MotorController
    import electrical.gps as GPS 
    import electrical.imu as IMU 
    import electrical.radio_module as RadioModule
    import serial
    # from engine.sensor_module import SensorModule # IMU + GPS testing

class Robot_State:
    """
    A class containing robot-specific information about the current state of a robot.

    Initializes the information for the robot.
    Parameters:
        x_pos: the x position of the robot, where (0,0) is the bottom left corner of the grid with which the Robot's related Mission was initialized
        y_pos: the y position of the robot where (0,0) is the bottom left corner of the grid with which the Robot's related Mission was initialized
        heading: the theta of the robot in radians, where North on the grid is equal to 0.
        epsilon: dictates the turning radius of the robot. Lower epsilon results in tighter turning radius.
        max_velocity: the maximum velocity of the robot
        radius: the radius of the robot
    Preconditions:
        x_pos is an int within the bounds of the user-defined traversal grid
        y_pos is an int within the bounds of the user-defined traversal grid
        heading is a float
        epsilon is a float
        max_velocity is a float
        radius is a float
    """
    def __init__(self, x_pos, y_pos, heading, epsilon, max_velocity, radius):
        """
        Instance Attributes:
            epsilon: dictates the turning radius of the robot. Lower epsilon results in tighter turning radius.
            max_velocity: the maximum velocity of the robot
            radius: the radius of the robot
            width: width of the robot in cm
            front_ultrasonic: the ultrasonic at the front of the robot, used for detecting obstacles
            lf_ultrasonic: the ultrasonic at the front of the left side of the robot, used for boundary following
            lb_ultrasonic: the ultrasonic at the back of the left side of the robot, used for boundary following
            rf_ultrasonic: the ultrasonic at the front of the right side of the robot, used for boundary following
            rb_ultrasonic: the ultrasonic at the back of the right side of the robot, used for boundary following
            is_sim: False if the physical robot is being used, True otherwise
            should_store_data: False if csv data should not be stored, True otherwise
            position_kp: the proportional factor of the position PID
            position_ki: the integral factor of the position PID
            position_kd: the derivative factor of the position PID
            position_noise: the flat amount of noise added to the robot's phase on each localization step
            heading_kp: the proportional factor of the heading PID
            heading_ki: the integral factor of the heading PID
            heading_kd: the derivative factor of the heading PID
            heading_noise: ?
            phase: the phase of the robot
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
        # User defined parameters
        self.epsilon = epsilon
        self.max_velocity = max_velocity
        self.radius = radius

        # Flags
        self.is_sim = not is_raspberrypi()
        self.should_store_data = False
        self.phase = Phase.SETUP
        self.avoid_obstacle = False
        
        # Constants
        self.width = 700
        self.time_step = 1
        self.position_kp = 1
        self.position_ki = 0
        self.position_kd = 0
        self.position_noise = 0
        self.heading_kp = 1
        self.heading_ki = 0
        self.heading_kd = 0
        self.heading_noise = 0
        self.move_dist = 0.5
        self.turn_angle = 3
        self.turn_angle = self.turn_angle / self.time_step # dividing by time_step ignores the effect of time_step on absolute
        self.init_threshold = 1
        self.goal_threshold = 1
        self.noise_margin = 1
        self.front_sensor_offset = 0  # TODO: replace this with how far offset the sensor is to the front of the robot
        self.max_sensor_range = 600
        self.sensor_measuring_angle = 75
        self.width_margin = 1  # TODO: replace this with actual margin
        self.threshold_distance = ((self.width + self.width_margin) / 2) / math.cos(math.radians((180 - self.sensor_measuring_angle) / 2))
        self.detect_obstacle_range = min(self.threshold_distance, self.max_sensor_range)  # set ultrasonic detection range

        # Measurements
        self.state = np.array([[x_pos], [y_pos], [heading]])
        self.truthpose = np.transpose(np.array([[x_pos], [y_pos], [heading]]))
        self.plastic_level = 0
        self.battery = 100
        self.acceleration = [0, 0, 0]
        self.magnetic_field = [0, 0, 0]
        self.gyro_rotation = [0, 0, 0]
        self.init_gps = (0, 0)
        self.gps_data = (0, 0)
        self.imu_data = None  # will be filled by execute_setup
        self.linear_v = 0
        self.angular_v = 0
        self.dist_to_goal = 0
        self.prev_phase = self.phase
        self.goal_location = (0, 0)
       
        # Sensors
        self.motor_controller = None
        self.robot_radio_session = None
        self.gps = None
        self.imu = None
        self.ekf = None
        self.front_ultrasonic = None
        self.lf_ultrasonic = None
        self.lb_ultrasonic = None
        self.rf_ultrasonic = None
        self.rb_ultrasonic = None

        if not self.is_sim:
            self.motor_controller = MotorController(wheel_radius = 0, vm_load1 = 1, vm_load2 = 1, L = 0, R = 0, is_sim = self.is_sim)
            self.robot_radio_session = RadioModule(serial.Serial('/dev/ttyS0', 57600)) 
            self.gps = GPS(serial.Serial('/dev/ttyACM0', 19200, timeout=5), is_sim = self.is_sim) 
            self.imu = IMU(init_i2c = busio.I2C(board.SCL, board.SDA), is_sim = self.is_sim) 
        

