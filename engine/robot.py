import threading

import numpy as np
import math
from enum import Enum
from electrical import motor_controller
from engine.kinematics import integrate_odom, feedback_lin, limit_cmds, get_vincenty_x, get_vincenty_y
from engine.pid_controller import PID
from csv_files.csv_util import write_state_to_csv, write_phase_to_csv
import time

from engine.is_raspberrypi import is_raspberrypi
if is_raspberrypi():
    from electrical.motor_controller import MotorController
    import electrical.gps as GPS
    import electrical.imu as IMU
    import electrical.radio_module as RadioModule
    import serial

from constants.definitions import *

from engine.ekf import LocalizationEKF
from engine.sensor_module import SensorModule
from constants.definitions import ENGINEERING_QUAD

from enum import Enum
import time
import sys


class Phase(Enum):
    """
    An enumeration of different robot phases.
    """
    SETUP = 1
    TRAVERSE = 2
    AVOID_OBSTACLE = 3
    RETURN = 4
    DOCKING = 5
    COMPLETE = 6
    FAULT = 7


class Robot:
    """
    A class whose objects contain robot-specific information, and methods to execute individual phases.

    Initializes the robot with the given position and heading.
    Parameters:
    state = Robot's state, np.array
        state contains the robot's x position, y position, and heading
    phase = 'collect' when traversing through grid, 'return' when returning to
        base station
    is_sim = False when running code with physical robot, True otherwise
    Preconditions:
    position is list of size 2
    heading is in range [0..359]
    """

    def __init__(self, x_pos, y_pos, heading, epsilon, max_v, radius, is_sim=True, is_store=False, width=700, front_ultrasonic=None,
                 lf_ultrasonic=None, lb_ultrasonic=None, rf_ultrasonic=None, rb_ultrasonic=None,
                 position_kp=1, position_ki=0, position_kd=0, position_noise=0, heading_kp=1, heading_ki=0,
                 heading_kd=0, heading_noise=0, init_phase=1, time_step=1, move_dist=.5, turn_angle=3,
                 plastic_weight=0, use_ekf=False, init_gps=(0, 0), gps_data=(0, 0), imu_data=None, ekf_var=None,
                 gps=None, imu=None, init_threshold=1, goal_threshold=1, noise_margin=1, motor_controller=None):
        """
        Arguments:
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
        self.state = np.array([[x_pos], [y_pos], [heading]])
        self.truthpose = np.transpose(np.array([[x_pos], [y_pos], [heading]]))
        self.is_sim = not is_raspberrypi()
        self.is_store = is_store
        self.phase = Phase(init_phase)
        self.epsilon = epsilon
        self.max_velocity = max_v
        self.radius = radius
        self.time_step = time_step
        self.position_kp = position_kp
        self.position_ki = position_ki
        self.position_kd = position_kd
        self.position_noise = position_noise
        self.heading_kp = heading_kp
        self.heading_ki = heading_ki
        self.heading_kd = heading_kd
        self.heading_noise = heading_noise
        self.move_dist = move_dist
        # dividing by time_step ignores the effect of time_step on absolute
        self.turn_angle = turn_angle / time_step
        self.plastic_weight = plastic_weight
        self.battery = 100  # TEMPORARY
        self.acceleration = [0, 0, 0]  # TEMPORARY
        self.magnetic_field = [0, 0, 0]  # TEMPORARY
        self.gyro_rotation = [0, 0, 0]  # TEMPORARY
        self.init_gps = init_gps
        self.gps_data = gps_data
        self.imu_data = imu_data  # will be filled by execute_setup
        self.ekf_var = ekf_var
        self.gps = gps
        self.imu = imu
        self.mc = motor_controller
        self.linear_v = 0
        self.angular_v = 0
        self.width = width
        self.avoid_obstacle = False  # boolean that determines if we should avoid obstacles
        self.front_ultrasonic = front_ultrasonic
        self.lf_ultrasonic = lf_ultrasonic
        self.lb_ultrasonic = lb_ultrasonic
        self.rf_ultrasonic = rf_ultrasonic
        self.rb_ultrasonic = rb_ultrasonic
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
        if not self.is_sim:
            self.motor_controller = MotorController(
                self, wheel_radius=0, vm_load1=1, vm_load2=1, L=0, R=0)
            self.robot_radio_session = RadioModule(
                serial.Serial('/dev/ttyS0', 57600))
            self.gps = GPS(serial.Serial('/dev/ttyACM0', 19200, timeout=5))
            self.imu = IMU(busio.I2C(board.SCL, board.SDA))

        self.loc_pid_x = PID(
            Kp=self.position_kp, Ki=self.position_ki, Kd=self.position_kd, target=0, sample_time=self.time_step,
            output_limits=(None, None)
        )

        self.loc_pid_y = PID(
            Kp=self.position_kp, Ki=self.position_ki, Kd=self.position_kd, target=0, sample_time=self.time_step,
            output_limits=(None, None)
        )

        self.head_pid = PID(
            Kp=self.heading_kp, Ki=self.heading_ki, Kd=self.heading_kd, target=0, sample_time=self.time_step,
            output_limits=(None, None)
        )

        # TODO: wrap in try/except (error when calling execute_setup_test.py)
        # write in csv
        if self.is_store:
            write_phase_to_csv(self.phase)

    def update_ekf_step(self):
        zone = ENGINEERING_QUAD  # Used for GPS visualization, make this not hard-coded
        # self.ekf_var.update_step(self.ekf_var.mu, self.ekf_var.sigma, sensor_module.get_measurement(self.init_gps))
        self.gps_data = (self.gps.get_gps()["long"], self.gps.get_gps()["lat"])
        self.imu_data = self.imu.get_gps()
        x, y = get_vincenty_x(
            self.init_gps, self.gps_data), get_vincenty_y(self.init_gps, self.gps_data)
        heading = math.degrees(math.atan2(
            self.imu_data["mag"]["y"], self.imu_data["mag"]["x"]))

        measurements = np.array([[x], [y], [heading]])

        self.ekf_var.update_step(
            self.ekf_var.mu, self.ekf_var.sigma, measurements)
        new_x = self.ekf_var.mu[0][0]
        new_y = self.ekf_var.mu[1][0]
        new_heading = self.ekf_var.mu[2][0]
        new_state = np.array([[new_x], [new_y], [new_heading]])
        return new_state

    def travel(self, velocity, omega):
        # Moves the robot with both linear and angular velocity
        self.state = np.round(integrate_odom(
            self.state, velocity, omega), 3)
        # if it is a simulation,
        if self.is_sim:
            self.truthpose = np.append(
                self.truthpose, np.transpose(self.state), 0)

    def move_forward(self, dist):
        # Moves robot forward by distance dist
        # dist is in meters
        new_x = self.state[0] + dist * math.cos(self.state[2]) * self.time_step
        new_y = self.state[1] + dist * math.sin(self.state[2]) * self.time_step
        self.state[0] = np.round(new_x, 3)
        self.state[1] = np.round(new_y, 3)

        if self.is_sim:
            self.truthpose = np.append(
                self.truthpose, np.transpose(self.state), 0)

    def move_to_target_node(self, target, allowed_dist_error, database):
        """
        Moves robot to target + or - allowed_dist_error
        Arguments:
            target: target coordinates in the form (latitude, longitude)
            allowed_dist_error: the maximum distance in meters that the robot 
            can be from a node for the robot to have "visited" that node
            database: 
        """
        predicted_state = self.state  # this will come from Kalman Filter

        # location error (in meters)
        distance_away = self.calculate_dist(target, predicted_state)

        while distance_away > allowed_dist_error:
            if self.is_sim:
                # Adding simulated noise to the robot's state based on gaussian distribution
                self.state[0] = np.random.normal(
                    self.state[0], self.position_noise)
                self.state[1] = np.random.normal(
                    self.state[1], self.position_noise)

            # Error in terms of latitude and longitude, NOT meters
            x_coords_error = target[0] - self.state[0]
            y_coords_error = target[1] - self.state[1]

            x_vel = self.loc_pid_x.update(x_coords_error)
            y_vel = self.loc_pid_y.update(y_coords_error)

            cmd_v, cmd_w = feedback_lin(
                predicted_state, x_vel, y_vel, self.epsilon)

            # clamping of velocities:
            (limited_cmd_v, limited_cmd_w) = limit_cmds(
                cmd_v, cmd_w, self.max_velocity, self.radius)
            # self.linear_v = limited_cmd_v[0]
            # self.angular_v = limited_cmd_w[0]

            if self.is_sim:
                # this is just simulating movement:
                self.travel(self.time_step * limited_cmd_v[0],
                            self.time_step * limited_cmd_w[0])
            else:
                self.motor_controller.spin_motors(
                    limited_cmd_w[0], limited_cmd_v[0])
                # TODO: sleep??

            if not self.is_sim:
                self.state = self.update_ekf_step()

            # Get state after movement:
            predicted_state = self.state  # this will come from Kalman Filter

            if self.is_sim and self.is_store:
                # TODO: Update to use databse information
                # FOR GUI: writing robot location and mag heading in CSV
                write_state_to_csv(predicted_state)
                time.sleep(0.001)  # Delays calculation for GUI map

            # FOR DATABASE: updating our database with new predicted state
            # TODO: can the code above be simplified / use the database instead?
            database.update_data(
                "state", predicted_state[0], predicted_state[1], predicted_state[2])

            # location error (in meters)
            distance_away = self.calculate_dist(target, predicted_state)

    def write_to_csv(self, predicted_state):
        with open(CSV_PATH + '/datastore.csv', 'a') as fd:
            fd.write(
                str(predicted_state[0])[1:-1] + ',' + str(predicted_state[1])[1:-1] + ',' + str(predicted_state[2])[1:-1] + '\n')
        time.sleep(0.001)

    def turn_to_target_heading(self, target_heading, allowed_heading_error, database):
        """
        Turns robot in-place to target heading + or - allowed_heading_error, utilizing heading PID.

        Arguments:
            target_heading: the heading in radians the robot should approach at the end of in-place rotation.
            allowed_heading_error: the maximum error in radians a robot can have to target heading while turning in
                place.
        """

        predicted_state = self.state  # this will come from Kalman Filter

        abs_heading_error = abs(target_heading - float(predicted_state[2]))

        while abs_heading_error > allowed_heading_error:
            if self.is_sim:
                self.state[2] = np.random.normal(
                    self.state[2], self.heading_noise)
            else:
                self.state = self.update_ekf_step()
            theta_error = target_heading - self.state[2]
            w = self.head_pid.update(theta_error)  # angular velocity
            _, limited_cmd_w = limit_cmds(0, w, self.max_velocity, self.radius)

            if self.is_sim:
                self.travel(0, self.time_step * limited_cmd_w)
            else:
                self.motor_controller.spin_motors(limited_cmd_w, 0)

            # Get state after movement:
            predicted_state = self.state  # this will come from Kalman Filter

            database.update_data(
                "state", self.state[0], self.state[1], self.state[2])

            abs_heading_error = abs(target_heading - float(predicted_state[2]))

    def turn(self, turn_angle):
        """
        Hardcoded in-place rotation for testing purposes. Does not use heading PID. Avoid using in physical robot.
        """
        # Turns robot, where turn_angle is given in radians
        clamp_angle = (self.state[2] + (turn_angle *
                                        self.time_step)) % (2 * math.pi)
        self.state[2] = np.round(clamp_angle, 3)
        if self.is_sim:
            self.truthpose = np.append(
                self.truthpose, np.transpose(self.state), 0)

    def get_state(self):
        return self.state

    def print_current_state(self):
        # Prints current robot state
        print('pos: ' + str(self.state[0:2]))
        print('heading: ' + str(self.state[2]))

    def execute_setup(self, radio_session, gps, imu, motor_controller):
        gps_setup = gps.setup()
        imu_setup = imu.setup()
        radio_session.setup_robot()
        motor_controller.setup(self.is_sim)

        zone = ENGINEERING_QUAD  # Used for GPS visualization, make it a parameter
        self.init_gps = (gps.get_gps()["long"], gps.get_gps()["lat"])
        self.imu_data = imu.get_gps()
        x_init, y_init = (0, 0)
        heading_init = math.degrees(math.atan2(
            self.imu_data["mag"]["y"], self.imu_data["mag"]["x"]))

        # mu is meters from start position (bottom left position facing up)
        mu = np.array([[x_init], [y_init], [heading_init]])

        # confidence of mu, set it to high initially b/c not confident, algo brings it down
        sigma = np.array([[10, 0, 0], [0, 10, 0], [0, 0, 10]])
        self.ekf_var = LocalizationEKF(mu, sigma)
        self.gps = gps
        self.imu = imu
        self.motor_controller = motor_controller
        if (radio_session.connected and gps_setup and imu_setup):
            obstacle_avoidance = threading.Thread(
                target=self.track_obstacle, daemon=True)
            obstacle_avoidance.start()  # spawn thread to monitor obstacles
            self.set_phase(Phase.TRAVERSE)

    def execute_traversal(self, unvisited_waypoints, allowed_dist_error, base_station_loc, control_mode, time_limit,
                          roomba_radius, database):
        if control_mode == 4:  # Roomba mode
            self.traverse_roomba(base_station_loc, time_limit, roomba_radius)
        else:
            self.traverse_standard(unvisited_waypoints,
                                   allowed_dist_error, database)

    def traverse_standard(self, unvisited_waypoints, allowed_dist_error, database):
        """ Move the robot by following the traversal path given by [unvisited_waypoints].
            Args:
                unvisited_waypoints ([Node list]): GPS traversal path in terms of meters for the current grid.
                allowed_dist_error (Double): the maximum distance in meters that the robot can be from a node for the
                    robot to have "visited" that node
            Returns:
                unvisited_waypoints ([Node list]): GPS traversal path in terms of meters for the current grid.
        """
        while unvisited_waypoints:
            curr_waypoint = unvisited_waypoints[0].get_m_coords()
            # TODO: add obstacle avoidance support
            # TODO: add return when tank is full, etc
            # TODO: add turn_to_target_heading
            self.move_to_target_node(
                curr_waypoint, allowed_dist_error, database)
            unvisited_waypoints.popleft()
            if self.avoid_obstacle:
                self.set_phase(Phase.AVOID_OBSTACLE)
                self.goal_location = curr_waypoint
                self.prev_phase = Phase.TRAVERSE
                return unvisited_waypoints

        self.set_phase(Phase.RETURN)
        return unvisited_waypoints

    def traverse_roomba(self, base_station_loc, time_limit, roomba_radius):
        """ Move the robot in a roomba-like manner.
            Args:
                base_station_loc (Tuple): The (x,y) location of the base station
                time_limit (Double): How long the robot will move using roomba mode
                roomba_radius (Double): Maximum radius from the base station that the robot can move
            Returns:
                None
        """
        # to add obstacle avoidance, can either change roomba to go to point on the circle in the direction of the
        # robot direction or use raw sensor value
        # can we guarantee that when giving a velocity that the robot will move at that velocity?
        dt = 0
        # TODO: battery_limit, time_limit, tank_capacity is full
        exit_boolean = False
        while not exit_boolean:

            curr_x = self.state[0]
            curr_y = self.state[1]
            new_x = curr_x + self.move_dist * \
                math.cos(self.state[2]) * self.time_step
            new_y = curr_y + self.move_dist * \
                math.sin(self.state[2]) * self.time_step
            next_radius = self.calculate_dist(base_station_loc, (new_x, new_y))
            is_detecting_obstacle = self.front_ultrasonic.distance() < self.detect_obstacle_range
            # if moving will cause the robot to move through the obstacle
            is_next_timestep_blocked = next_radius < self.detect_obstacle_range
            # sensor should not detect something in the robot
            if is_detecting_obstacle < self.front_sensor_offset:
                self.set_phase(Phase.FAULT)
                return None
            if (next_radius > roomba_radius) or (is_detecting_obstacle and is_next_timestep_blocked):
                if self.is_sim:
                    self.move_forward(-self.move_dist)
                    self.turn(self.turn_angle)
                else:
                    self.motor_controller.motors(0, 0)
                    # TODO: change this to pid or time based. NEED TO MAKE SURE ROBOT DOESN'T BREAK WHEN GOING FROM
                    #  POS VEL TO NEG VEL IN A SHORT PERIOD OF TIME -> ramp down prob
            else:
                if self.is_sim:
                    self.move_forward(self.move_dist)
                else:
                    # TODO: determine what vel to run this at
                    self.motor_controller.motors(0, 0)
            dt += 1
            exit_boolean = (dt > time_limit)
        self.set_phase(Phase.COMPLETE)  # TODO: CHANGE the next phase to return
        return None

    def set_phase(self, new_phase):
        self.phase = new_phase
        if self.is_store:
            write_phase_to_csv(self.phase)

    def track_obstacle(self):
        """ Continuously checks if there's an obstacle in the way.
            Will be executing on a separate, asynchronous thread.
            Precondition:
                Front ultrasonic sensor is mounted in the center front/x position of the robot (width/2)
        """
        # assuming front sensor is mounted in the center x position (width/2)
        counter = 0  # added for testing
        while True:
            if self.is_sim:
                ultrasonic_value_file = open(ROOT_DIR + '/tests/functionality_tests/csv/ultrasonic_values.csv',
                                             "r")  # added for testing
                content = ultrasonic_value_file.readlines()
                ultrasonic_value_file.close()
                try:
                    line = content[counter]
                    counter += 1
                    curr_ultrasonic_value = float(
                        (''.join(line.rstrip('\n')).strip('()').split(', '))[0])
                except IndexError:
                    print("no more sensor data")
                    break
            else:
                curr_ultrasonic_value = self.front_ultrasonic.distance()
                if curr_ultrasonic_value < self.front_sensor_offset:
                    self.set_phase(Phase.FAULT)
                    return None
            if (self.phase == Phase.TRAVERSE) or (self.phase == Phase.RETURN) or (self.phase == Phase.DOCKING) or (
                    self.phase == Phase.AVOID_OBSTACLE):
                if curr_ultrasonic_value < self.detect_obstacle_range:
                    # Note: didn't check whether we can reach goal before contacting obstacle because obstacle
                    # detection does not detect angle, so obstacle could be calculated to be falsely farther away than
                    # the goal. Not optimal because in cases, robot will execute boundary following when it can reach
                    # goal
                    self.dist_to_goal = self.calculate_dist(
                        self.goal_location, self.state)
                    self.avoid_obstacle = True
                    if self.is_sim:
                        with open(ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv', 'a') as fd:
                            fd.write("Avoid" + '\n')
                else:
                    self.avoid_obstacle = False
                    if self.is_sim:
                        with open(ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv', 'a') as fd:
                            fd.write("Not Avoid" + '\n')
            if curr_ultrasonic_value < 0:
                # value should not go below 0; sensor is broken
                self.set_phase(Phase.FAULT)
                if self.is_sim:
                    with open(ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv', 'a') as fd:
                        fd.write("Fault" + '\n')

        # time.sleep(10)  # don't hog the cpu

    def is_on_line(self, p1, p2):
        """ Checks if the robot is on the line defined by the two points p1 and p2
            Args:
                p1 (Tuple): An (x, y) point on the line
                p2 (Tuple): Another (x, y) point on the line
        """
        tolerance = 2  # threshold from current y position to desired y position on the line where we interpret y position to still be on the line
        slope = (p2[1] - p1[1])/(p2[0] - p1[0])  # m = (y2 - y1)/(x2 - x1)
        desiredy = slope * (self.x - p1[0]) + p1[1]  # y - y1 = m(x - x1)
        difference = self.y_pos - desiredy
        return abs(difference) < tolerance

    def execute_avoid_obstacle(self):
        """ Execute bug 2 (straight line) obstacle avoidance algorithm
            Returns:
                prev_phase (Phase)
        """
        # Note: init_threshold is arbitrary, set later. sometimes location reading will be inaccurate or not frequent
        #  enough to read that we've arrived back to the init x y pos. make it small enough that robot actually leaves
        #  init_threshold at some time during boundary following or add timeout to branch making gate = True
        init_threshold = 3
        # how far robot can be from goal for it to be at goal; goal is where the robot wanted to go when there isn't an obstacle
        goal_threshold = 3
        init_x = self.x_pos
        init_y = self.y_pos
        init_pos = (init_x, init_y)

        # last position of the robot when it's still on the (line from its initial position to its goal positions)
        last_pos_on_line = init_pos
        has_left_init_thresh = False
        has_traversed_boundary = False
        curr_dist_to_goal = self.calculate_dist(self.goal_location, curr_pos)
        init_dist_to_goal = curr_dist_to_goal
        while True:
            if self.phase == Phase.FAULT:  # fault has priority over obstacle avoidance
                return None
            curr_pos = (self.x_pos, self.y_pos)
            curr_dist_to_goal = self.calculate_dist(
                self.goal_location, curr_pos)

            dist_from_init = self.calculate_dist(curr_pos, init_pos)
            is_on_line = self.is_on_line(self.goal_location, init_pos)
            new_dist_to_goal = self.calculate_dist(
                self.goal_location, curr_pos)
            if dist_from_init > init_threshold:
                has_left_init_thresh = True
            if curr_dist_to_goal < goal_threshold:  # exits obstacle avoidance if robot close to goal
                self.set_phase(self.prev_phase)
                return None
            elif has_traversed_boundary and has_left_init_thresh:
                self.set_phase(Phase.FAULT)  # cannot reach goal
                return None
            elif is_on_line and (new_dist_to_goal < init_dist_to_goal):  # bug 2
                # without this, the robot could move closer to the goal (on a slant, not directly towards the goal)
                #   but still be on the line, making the robot keep exiting obstacle avoidance when its effectively in the same position as before
                threshold_to_recalculate_on_line = 3
                # need to tune. Want to be close to is_on_line threshold beecause that's the reason why we have this
                if self.calculate_dist(curr_pos, last_pos_on_line) > threshold_to_recalculate_on_line:
                    self.set_phase(self.prev_phase)
                    return None
            else:
                self.execute_boundary_following(0)  # boundary following here
            if has_left_init_thresh:
                # we dont want the program to think that the boundary is untraversable
                #   when the robot is still in the threshold from where it started obstacle avoidance
                has_traversed_boundary = dist_from_init < init_threshold
            time.sleep(10)  # don't hog the cpu

    def execute_boundary_following(self, min_dist):
        '''
        Currently, algorithm only supports using the right side sensors of the robot for boundary following.
        When encountering an obstacle, robot will begin boundary following by going clockwise around the object.
        This was arbitrarily decided since currently there is no better way to determine which direction the robot should
        navigate around the obstacle.
        Robot will always begin boundary following by going clockwise (turning left when encountering an obstacle)
        1. Determine if robot is parallel using the ultrasonic sensor data
            * Three cases:
                1. Front right sensor is farther away from the obstacle than the back right sensor
                    * Turn right until sensor inputs are equalized (front of the robot will turn closer toward the obstacle)
                2. Front right sensor is closer to the obstacle than the back right sensor
                    *  Turn left until sensor inputs are equalized (front of the robot will turn away from the obstacle)
                3. Both right-side sensors displaying the same value
                    * With a margin of error accounting for not uniform obstacles
        2. Once both sensors are displaying the same values, move forward (boundary following)
        3. Exits boundary following when conditions met in execute_avoid_obstacle() method
        '''
        front_dist = self.rf_ultrasonic.distance()
        back_dist = self.rb_ultrasonic.distance()
        margin = 1
        # Plus or minus distance value used to calculate if robot is parallel to an non-uniform object
        # (i.e. not a flat surface). forwardRightSensorReading - backRightSensorReading < margin means
        # robot is parallel to object.
        forward_dist = 0.01  # Distance moved forward by robot in one iteration of this method
        turn_angle = math.pi / 90
        # Turn angle of robot in one iteration of this method. Robot turns turn_angle if
        # forwardRightSensorReading > backRightSensorReading and -turn_angle if
        # forwardRightSensorReading < backRightSensorReading

        if math.abs(front_dist - back_dist) < margin:
            self.move_forward(forward_dist)
        elif front_dist > back_dist:
            self.turn(turn_angle)
        else:
            self.turn(-1 * turn_angle)

    # to do:
        # determine ccw or cw
        # implement main algorithm to make sure robot is parallel
        # test main algorithm
        # add cases for smooth turning
        #   make sure to take into account width and length of robot and ultrasonic sensor value on both sides
        #   make sure to take into account situation when the gap you are turning into is smaller than the robot width
        # add cases for obstacles when boundary following
        #   gap in wall but robot cannot fit
        # add cases for sharp turns
    # future to do:
        # add dp to quit when following boundary

    def execute_return(self, base_loc, base_angle, allowed_docking_pos_error, allowed_heading_error, database):
        """
        Returns robot to base station when robot is in RETURN phase and switches to DOCKING.

        Arguments:
            base_loc: location of the base station in meters in the form (x, y)
            base_angle: which direction the base station is facing in terms of unit circle (in radians)
            allowed_docking_pos_error: the maximum distance in meters the robot can be from "ready to dock" position
                before it can start docking (must be small)
            allowed_heading_error: the maximum error in radians a robot can have to target heading while turning in
                place.
        """
        docking_dist_to_base = 1.0  # how close the robot should come to base before starting DOCKING
        dx = docking_dist_to_base * math.cos(base_angle)
        dy = docking_dist_to_base * math.sin(base_angle)
        target_loc = (base_loc[0] + dx, base_loc[1] + dy)

        # TODO: add obstacle avoidance support
        self.move_to_target_node(
            target_loc, allowed_docking_pos_error, database)

        # Face robot towards base station
        target_heading = base_angle + math.pi
        self.turn_to_target_heading(
            target_heading, allowed_heading_error, database)

        # RETURN phase complete:
        self.set_phase(Phase.DOCKING)

    def execute_docking(self):
        # TODO: add traverse to doc at base station in case mission didnt finish
        self.set_phase(Phase.COMPLETE)  # temporary for simulation purposes

    def calculate_dist(self, init_pos, final_pos):
        return math.sqrt((final_pos[0] - init_pos[0]) ** 2 + (final_pos[1] - init_pos[1]) ** 2)
