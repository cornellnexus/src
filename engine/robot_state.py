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
        heading is a float in range [0..359]
        epsilon is a float
        max_velocity is a float
        radius is a float
    """

    def __init__(self, **kwargs):
        """
        Instance Attributes:
            FLAGS
            is_sim: False if the physical robot is being used, True otherwise
            should_store_data: False if csv data should not be stored, True otherwise
            phase: the phase of the robot
            avoid_obstacle: True if the robot is currently avoiding an obstacle, False otherwise

            CONSTANTS
            width: width of the robot in cm
            time_step: the amount of time that passes between each feedback loop cycle, should only be used if is_sim
                is True
            position_kp: the proportional factor of the position PID
            position_ki: the integral factor of the position PID
            position_kd: the derivative factor of the position PID
            position_noise: the flat amount of noise added to the robot's phase on each localization step
            heading_kp: the proportional factor of the heading PID
            heading_ki: the integral factor of the heading PID
            heading_kd: the derivative factor of the heading PID
            heading_noise: ?
            move_dist: the distance in meters that the robot moves per time dt
            epsilon: dictates the turning radius of the robot. Lower epsilon results in tighter turning radius.
            max_velocity: the maximum velocity of the robot
            radius: the radius of the robot
            turn_angle: the angle in radians that the robot turns per time dt regardless of time step
            front_sensor_offset:
            max_sensor_range:
            sensor_measuring_angle:
            width_margin:
            threshold_distance:
            detect_obstacle_range:

            MEASUREMENTS
            state: Robot's state, np.array state contains the robot's x position, y position, and heading
            truthpose:
            plastic_level: the percentage of the trash the robot has collected (25%, 50%, 75%, 100%)
            battery:
            acceleration:
            magnetic_field:
            gyro_rotation:
            init_gps:
            gps_data:
            imu_data:
            linear_v:
            angular_v:
            dist_to_goal:
            prev_phase:
            goal_location:

            SENSORS
            motor_controller:
            robot_radio_session:
            gps:
            imu:
            ekf:
            front_ultrasonic: the ultrasonic at the front of the robot, used for detecting obstacles
            lf_ultrasonic: the ultrasonic at the front of the left side of the robot, used for boundary following
            lb_ultrasonic: the ultrasonic at the back of the left side of the robot, used for boundary following
            rf_ultrasonic: the ultrasonic at the front of the right side of the robot, used for boundary following
            rb_ultrasonic: the ultrasonic at the back of the right side of the robot, used for boundary following

        """
        # TODO: Fill in missing spec for attributes above

        # FLAGS
        # If false, only uses GPS and IMU; else, uses EKF
        self.using_ekf = False
        self.is_sim = kwargs.get("is_sim", not is_raspberrypi())
        self.should_store_data = kwargs.get("should_store_data", False)
        self.phase = Phase(kwargs.get("phase", Phase.SETUP))
        self.avoid_obstacle = kwargs.get("avoid_obstacle", False)
        self.is_roomba_obstacle = kwargs.get("roomba_obstacle", False)

        # CONSTANTS
        self.width = kwargs.get("width", 700)
        self.length = kwargs.get("length", 700)  # placeholder, in meters
        self.time_step = kwargs.get("time_step", 1)
        self.position_kp = kwargs.get("position_kp", 1)
        self.position_ki = kwargs.get("position_ki", 0)
        self.position_kd = kwargs.get("position_kd", 0)
        self.position_noise = kwargs.get("position_noise", 0)
        self.heading_kp = kwargs.get("heading_kp", 1)
        self.heading_ki = kwargs.get("heading_ki", 0)
        self.heading_kd = kwargs.get("heading_kd", 0)
        self.heading_noise = kwargs.get("heading_noise", 0)
        # Note: user-defined parameter; defaulted to most common use case
        self.epsilon = kwargs.get("epsilon", 0.2)
        # Note: user-defined parameter; defaulted to most common use case
        self.max_velocity = kwargs.get("max_velocity", 0.5)
        # Note: user-defined parameter; defaulted to most common use case
        self.radius = kwargs.get("radius", 0.2)
        self.move_dist = kwargs.get("move_dist", 0.5)
        self.turn_angle = kwargs.get("turn_angle", 3)
        # dividing by time_step ignores the effect of time_step on absolute
        self.turn_angle = self.turn_angle / self.time_step
        # TODO: replace this with how far offset the sensor is to the front of the robot
        self.front_sensor_offset = kwargs.get("front_sensor_offset", 0)
        self.max_sensor_range = kwargs.get("max_sensor_range", 600)
        self.sensor_measuring_angle = kwargs.get("sensor_measuring_angle", 75)
        # TODO: replace this with actual margin
        self.width_margin = kwargs.get("width_margin", 1)
        self.threshold_distance = kwargs.get("threshold_distance", ((
            self.width + self.width_margin) / 2) / math.cos(math.radians((180 - self.sensor_measuring_angle) / 2)))
        self.detect_obstacle_range = kwargs.get("detect_obstacle_range", min(
            self.threshold_distance, self.max_sensor_range))  # set ultrasonic detection range

        # MEASUREMENTS
        self.state = kwargs.get("state", np.array(
            [[kwargs.get("xpos", 0)], [kwargs.get("ypos", 0)], [kwargs.get("heading", 0)]]))
        self.truthpose = kwargs.get("truthpose", np.transpose(np.array(
            [[kwargs.get("xpos", 0)], [kwargs.get("ypos", 0)], [kwargs.get("heading", 0)]])))
        self.plastic_level = kwargs.get("plastic_level", 0)
        self.battery = kwargs.get("battery", 100)
        self.acceleration = kwargs.get("acceleration", [0, 0, 0])
        self.magnetic_field = kwargs.get("magnetic_field", [0, 0, 0])
        self.gyro_rotation = kwargs.get("gyro_rotation", [0, 0, 0])
        self.init_gps = kwargs.get("init_gps", (0, 0))
        self.gps_data = kwargs.get("gps_data", (0, 0))
        # will be filled by execute_setup
        self.imu_data = kwargs.get("imu_data", None)
        self.linear_v = kwargs.get("linear_v", 0)
        self.angular_v = kwargs.get("angular_v", 0)
        self.prev_phase = kwargs.get("prev_phase", self.phase)
        self.goal_location = kwargs.get("goal_location", (0, 0))

        # SENSORS
        self.motor_controller = kwargs.get("motor_controller", None)
        self.robot_radio_session = kwargs.get("robot_radio_session", None)
        self.gps = kwargs.get("gps", None)
        self.imu = kwargs.get("imu", None)
        self.ekf = kwargs.get("ekf", None)

        # THREADS
        self.track_obstacle_thread = None

        # TODO: GPS, IMU, RF module and Motor Controller are also re-initialized in robot.execute_setup
        # We should pick whether we want to initialize the attributes here or in robot.execute_setup
        # (which is being passed from Mission, need to whether sensors are part of mission vs robot)
        if not self.is_sim:
            self.motor_controller = MotorController(
                wheel_radius=0, vm_load1=1, vm_load2=1, L=0, R=0, is_sim=self.is_sim)
            self.robot_radio_session = RadioModule(
                serial.Serial('/dev/ttyS0', 57600))
            self.gps = GPS(serial.Serial('/dev/ttyACM0', 19200,
                           timeout=5), is_sim=self.is_sim)
            self.imu = IMU(init_i2c=busio.I2C(
                board.SCL, board.SDA), is_sim=self.is_sim)
