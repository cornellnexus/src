import math
import numpy as np
from engine.phase import Phase
from engine.control_mode import ControlMode
from engine.is_raspberrypi import is_raspberrypi
from engine.pid_controller import PID


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
            using_ekf: False if are only using GPS/IMU, True if we are using EKF
            is_sim: False if the physical robot is being used, True otherwise
            store_data: False if csv data should not be stored, True otherwise
            phase: the phase of the robot
            enable_obstacle_avoidance: False if we want to pause obstacle avoidance

            CONSTANTS
            width: width of the robot in cm
            length: length of the robot in cm
            time_step: the amount of time that passes between each feedback loop cycle, should only be used if is_sim
                is True
            control_mode: parameter to designate if we want to do lawnmower traversal or roomba traversal
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
            init_threshold: Radius in meters from initial position that will detect robot is back in initial position
                    This parameter needs to be tuned.
                    The reasoning behind this parameter is that sometimes location reading will be inaccurate or not frequent read
                    so the sensor thinks we're still at the init x y pos. We should make this small enough that robot actually leaves
                    init_threshold at some time during boundary following or add a timeout
            goal_threshold: Threshold in meters from goal that will be detected as reaching goal in obstacle avoidance.
                    Goal is where the robot wanted to go when there isn't an obstacle
            threshold_to_recalculate_on_line: threshold in meters that we dont want to recalculate whether the robot
                    is on the line to the goal.
                    This parameter needs to be tuned
                    Reasoning for this: without this condition, the robot could move closer to the goal (on a slant, not directly
                    towards the goal) but still be on the line, making the robot keep exiting obstacle avoidance when its effectively
                    in the same position as before

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
        """
        # TODO: Fill in missing spec for attributes above

        # FLAGS
        # If false, only uses GPS and IMU; else, uses EKF
        self.using_ekf = False
        self.is_sim = kwargs.get("is_sim", not is_raspberrypi())
        self.should_store_data = kwargs.get("store_data", False)
        self.phase = Phase(
            kwargs.get("phase", Phase.TRAVERSE if self.is_sim else Phase.SETUP)
        )
        self.enable_obstacle_avoidance = kwargs.get("enable_obstacle_avoidance", True)

        # CONSTANTS
        self.width = kwargs.get("width", 700)
        self.length = kwargs.get("length", 700)
        self.time_step = kwargs.get("time_step", 1)
        self.control_mode = kwargs.get("control_mode", ControlMode.LAWNMOWER)
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
        self.turn_angle = kwargs.get("turn_angle", 5 * math.pi / 7)
        # TODO: replace this with how far offset the sensor is to the front of the robot
        self.front_sensor_offset = kwargs.get("front_sensor_offset", 0)
        self.max_sensor_range = kwargs.get("max_sensor_range", 600)
        self.sensor_measuring_angle = kwargs.get("sensor_measuring_angle", 75)
        # TODO: replace this with actual margin
        self.width_margin = kwargs.get("width_margin", 1)
        self.threshold_distance = kwargs.get(
            "threshold_distance",
            ((self.width + self.width_margin) / 2)
            / math.cos(math.radians((180 - self.sensor_measuring_angle) / 2)),
        )
        self.detect_obstacle_range = kwargs.get(
            "detect_obstacle_range", min(self.threshold_distance, self.max_sensor_range)
        )  # set ultrasonic detection range
        # Obstacle Avoidance Constants
        self.init_threshold = kwargs.get("init_threshold", 3.0)
        self.goal_threshold = kwargs.get("goal_threshold", 3.0)
        self.threshold_to_recalculate_on_line = kwargs.get(
            "threshold_to_recalculate_on_line", 3.0
        )

        # MEASUREMENTS
        self.state = kwargs.get(
            "state",
            np.array(
                [
                    [kwargs.get("xpos", 0)],
                    [kwargs.get("ypos", 0)],
                    [kwargs.get("heading", 0)],
                ]
            ),
        )
        self.truthpose = kwargs.get(
            "truthpose",
            np.transpose(
                np.array(
                    [
                        [kwargs.get("xpos", 0)],
                        [kwargs.get("ypos", 0)],
                        [kwargs.get("heading", 0)],
                    ]
                )
            ),
        )
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

        self.front_ultrasonic = kwargs.get("front_ultrasonic", None)
        self.lf_ultrasonic = kwargs.get("lf_ultrasonic", None)
        self.lb_ultrasonic = kwargs.get("lb_ultrasonic", None)
        self.rf_ultrasonic = kwargs.get("rf_ultrasonic", None)
        self.rb_ultrasonic = kwargs.get("rb_ultrasonic", None)

        # OBJECTS
        self.loc_pid_x = PID(
            Kp=self.position_kp,
            Ki=self.position_ki,
            Kd=self.position_kd,
            target=0,
            sample_time=self.time_step,
            output_limits=(None, None),
        )

        self.loc_pid_y = PID(
            Kp=self.position_kp,
            Ki=self.position_ki,
            Kd=self.position_kd,
            target=0,
            sample_time=self.time_step,
            output_limits=(None, None),
        )

        self.head_pid = PID(
            Kp=self.heading_kp,
            Ki=self.heading_ki,
            Kd=self.heading_kd,
            target=0,
            sample_time=self.time_step,
            output_limits=(None, None),
        )
