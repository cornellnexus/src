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
from constants.definitions import CSV_PATH
from engine.phase import Phase

from engine.ekf import LocalizationEKF
from engine.sensor_module import SensorModule
from constants.definitions import ENGINEERING_QUAD
# import electrical.gps as gps 
# import electrical.imu as imu 
# import electrical.radio_module as radio_module
# from electrical.motor_controller import MotorController

import time
import sys

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

    def __init__(self, robot_state):
        self.robot_state = robot_state

        self.loc_pid_x = PID(
            Kp=self.robot_state.position_kp, Ki=self.robot_state.position_ki, Kd=self.robot_state.position_kd, target=0, sample_time=self.robot_state.time_step,
            output_limits=(None, None)
        )

        self.loc_pid_y = PID(
            Kp=self.robot_state.position_kp, Ki=self.robot_state.position_ki, Kd=self.robot_state.position_kd, target=0, sample_time=self.robot_state.time_step,
            output_limits=(None, None)
        )

        self.head_pid = PID(
            Kp=self.robot_state.heading_kp, Ki=self.robot_state.heading_ki, Kd=self.robot_state.heading_kd, target=0, sample_time=self.robot_state.time_step,
            output_limits=(None, None)
        )

        # TODO: wrap in try/except (error when calling execute_setup_test.py)
        # write in csv
        if self.robot_state.is_store:
            write_phase_to_csv(self.robot_state.phase)

    def update_ekf_step(self):
        zone = ENGINEERING_QUAD  # Used for GPS visualization, make this not hard-coded
        # self.robot_state.ekf_var.update_step(self.robot_state.ekf_var.mu, self.robot_state.ekf_var.sigma, sensor_module.get_measurement(self.robot_state.init_gps))
        self.robot_state.gps_data = (self.robot_state.gps.get_gps()["long"], self.robot_state.gps.get_gps()["lat"])
        self.robot_state.imu_data = self.robot_state.imu.get_gps()
        x, y = get_vincenty_x(
            self.robot_state.init_gps, self.robot_state.gps_data), get_vincenty_y(self.robot_state.init_gps, self.robot_state.gps_data)
        heading = math.degrees(math.atan2(
            self.robot_state.imu_data["mag"]["y"], self.robot_state.imu_data["mag"]["x"]))

        measurements = np.array([[x], [y], [heading]])

        self.robot_state.ekf_var.update_step(
            self.robot_state.ekf_var.mu, self.robot_state.ekf_var.sigma, measurements)
        new_x = self.robot_state.ekf_var.mu[0][0]
        new_y = self.robot_state.ekf_var.mu[1][0]
        new_heading = self.robot_state.ekf_var.mu[2][0]
        new_state = np.array([[new_x], [new_y], [new_heading]])
        return new_state

    def travel(self, velocity, omega):
        # Moves the robot with both linear and angular velocity
        self.robot_state.state = np.round(integrate_odom(
            self.robot_state.state, velocity, omega), 3)
        # if it is a simulation,
        if self.robot_state.is_sim:
            self.robot_state.truthpose = np.append(
                self.robot_state.truthpose, np.transpose(self.robot_state.state), 0)
        else:
            imu_data = self.robot_state.imu.get_imu()
            self.robot_state.state[2] = math.degrees(math.atan2(imu_data["mag"][1], imu_data["mag"][0]))

    def move_forward(self, dist):
        # Moves robot forward by distance dist
        # dist is in meters
        new_x = self.robot_state.state[0] + dist * math.cos(self.robot_state.state[2]) * self.robot_state.time_step
        new_y = self.robot_state.state[1] + dist * math.sin(self.robot_state.state[2]) * self.robot_state.time_step
        self.robot_state.state[0] = np.round(new_x, 3)
        self.robot_state.state[1] = np.round(new_y, 3)

        if self.robot_state.is_sim:
            self.robot_state.truthpose = np.append(
                self.robot_state.truthpose, np.transpose(self.robot_state.state), 0)

    def move_to_target_node(self, target, allowed_dist_error, database):
        """
        Moves robot to target + or - allowed_dist_error
        Arguments:
            target: target coordinates in the form (latitude, longitude)
            allowed_dist_error: the maximum distance in meters that the robot 
            can be from a node for the robot to have "visited" that node
            database: 
        """
        predicted_state = self.robot_state.state  # this will come from Kalman Filter

        # location error (in meters)
        distance_away = self.calculate_dist(target, predicted_state)

        while distance_away > allowed_dist_error:
            if self.robot_state.is_sim:
                # Adding simulated noise to the robot's state based on gaussian distribution
                self.robot_state.state[0] = np.random.normal(
                    self.robot_state.state[0], self.robot_state.position_noise)
                self.robot_state.state[1] = np.random.normal(
                    self.robot_state.state[1], self.robot_state.position_noise)

            # Error in terms of latitude and longitude, NOT meters
            x_coords_error = target[0] - self.robot_state.state[0]
            y_coords_error = target[1] - self.robot_state.state[1]

            x_vel = self.loc_pid_x.update(x_coords_error)
            y_vel = self.loc_pid_y.update(y_coords_error)

            cmd_v, cmd_w = feedback_lin(
                predicted_state, x_vel, y_vel, self.robot_state.epsilon)

            # clamping of velocities:
            (limited_cmd_v, limited_cmd_w) = limit_cmds(
                cmd_v, cmd_w, self.robot_state.max_velocity, self.robot_state.radius)
            # self.linear_v = limited_cmd_v[0]
            # self.angular_v = limited_cmd_w[0]

            self.robot_state.linear_v = limited_cmd_v[0]
            self.robot_state.angular_v = limited_cmd_w[0]

            if self.robot_state.is_sim:
                # this is just simulating movement:
                self.travel(self.robot_state.time_step * limited_cmd_v[0],
                            self.robot_state.time_step * limited_cmd_w[0])
            else:
                self.robot_state.motor_controller.spin_motors(
                    limited_cmd_w[0], limited_cmd_v[0])
                # TODO: sleep??

            if not self.robot_state.is_sim:
                self.robot_state.state = self.update_ekf_step()

            # Get state after movement:
            predicted_state = self.robot_state.state  # this will come from Kalman Filter

            if self.robot_state.is_sim and self.robot_state.is_store:
                # TODO: Update to use databse information
                # FOR GUI: writing robot location and mag heading in CSV
                write_state_to_csv(predicted_state)
                time.sleep(0.001) # Delays calculation for GUI map

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

        predicted_state = self.robot_state.state  # this will come from Kalman Filter

        abs_heading_error = abs(target_heading - float(predicted_state[2]))

        while abs_heading_error > allowed_heading_error:
            if self.robot_state.is_sim:
                self.robot_state.state[2] = np.random.normal(self.robot_state.state[2], self.robot_state.heading_noise)
            else:
                self.robot_state.state = self.update_ekf_step()
            theta_error = target_heading - self.robot_state.state[2]
            w = self.head_pid.update(theta_error)  # angular velocity
            _, limited_cmd_w = limit_cmds(0, w, self.robot_state.max_velocity, self.robot_state.radius)

            self.travel(0, self.robot_state.time_step * limited_cmd_w)
            # sleep in real robot

            # Get state after movement:
            predicted_state = self.robot_state.state  # this will come from Kalman Filter

            database.update_data(
                "state", self.robot_state.state[0], self.robot_state.state[1], self.robot_state.state[2])

            abs_heading_error = abs(target_heading - float(predicted_state[2]))

    def turn(self, turn_angle):
        """
        Hardcoded in-place rotation for testing purposes. Does not use heading PID. Avoid using in physical robot.
        """
        # Turns robot, where turn_angle is given in radians
        clamp_angle = (self.robot_state.state[2] + (turn_angle *
                                        self.robot_state.time_step)) % (2 * math.pi)
        self.robot_state.state[2] = np.round(clamp_angle, 3)
        if self.robot_state.is_sim:
            self.robot_state.truthpose = np.append(
                self.robot_state.truthpose, np.transpose(self.robot_state.state), 0)

    def get_state(self):
        return self.robot_state.state

    def print_current_state(self):
        # Prints current robot state
        print('pos: ' + str(self.robot_state.state[0:2]))
        print('heading: ' + str(self.robot_state.state[2]))

    def execute_setup(self, radio_session, gps, imu, motor_controller):
        gps_setup = gps.setup()
        imu_setup = imu.setup()
        radio_session.setup_robot()
        motor_controller.setup(self.robot_state.is_sim)

        zone = ENGINEERING_QUAD  # Used for GPS visualization, make it a parameter
        self.robot_state.init_gps = (gps.get_gps()["long"], gps.get_gps()["lat"])
        self.robot_state.imu_data = imu.get_gps()
        x_init, y_init = (0, 0)
        heading_init = math.degrees(math.atan2(
            self.robot_state.imu_data["mag"]["y"], self.robot_state.imu_data["mag"]["x"]))

        # mu is meters from start position (bottom left position facing up)
        mu = np.array([[x_init], [y_init], [heading_init]])

        # confidence of mu, set it to high initially b/c not confident, algo brings it down
        sigma = np.array([[10, 0, 0], [0, 10, 0], [0, 0, 10]])
        self.robot_state.ekf_var = LocalizationEKF(mu, sigma)
        self.robot_state.gps = gps
        self.robot_state.imu = imu
        self.robot_state.motor_controller = motor_controller
        if (radio_session.connected and gps_setup and imu_setup):
            obstacle_avoidance = threading.Thread(target=self.track_obstacle, daemon=True)
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
            if self.robot_state.avoid_obstacle:
                self.set_phase(Phase.AVOID_OBSTACLE)
                self.robot_state.goal_location = curr_waypoint
                self.robot_state.prev_phase = Phase.TRAVERSE
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

            curr_x = self.robot_state.state[0]
            curr_y = self.robot_state.state[1]
            new_x = curr_x + self.robot_state.move_dist * math.cos(self.robot_state.state[2]) * self.robot_state.time_step
            new_y = curr_y + self.robot_state.move_dist * math.sin(self.robot_state.state[2]) * self.robot_state.time_step
            next_radius = self.calculate_dist(base_station_loc, (new_x, new_y))
            is_detecting_obstacle = self.robot_state.front_ultrasonic.distance() < self.robot_state.detect_obstacle_range
            # if moving will cause the robot to move through the obstacle
            is_next_timestep_blocked = next_radius < self.robot_state.detect_obstacle_range
            # sensor should not detect something in the robot
            if is_detecting_obstacle < self.robot_state.front_sensor_offset:
                self.set_phase(Phase.FAULT)
                return None
            if (next_radius > roomba_radius) or (is_detecting_obstacle and is_next_timestep_blocked):
                if self.robot_state.is_sim:
                    self.move_forward(-self.robot_state.move_dist)
                    self.turn(self.robot_state.turn_angle)
                else:
                    self.robot_state.motor_controller.motors(0, 0)
                    # TODO: change this to pid or time based. NEED TO MAKE SURE ROBOT DOESN'T BREAK WHEN GOING FROM
                    #  POS VEL TO NEG VEL IN A SHORT PERIOD OF TIME -> ramp down prob
            else:
                if self.robot_state.is_sim:
                    self.move_forward(self.robot_state.move_dist)
                else:
                    self.robot_state.motor_controller.motors(0, 0)  # TODO: determine what vel to run this at
            dt += 1
            exit_boolean = (dt > time_limit)
        self.set_phase(Phase.COMPLETE)  # TODO: CHANGE the next phase to return
        return None

    def set_phase(self, new_phase):
        self.robot_state.phase = new_phase
        if self.robot_state.is_store:
            write_phase_to_csv(self.robot_state.phase)

    def track_obstacle(self):
        """ Continuously checks if there's an obstacle in the way.
            Will be executing on a separate, asynchronous thread.
            Precondition:
                Front ultrasonic sensor is mounted in the center front/x position of the robot (width/2)
        """
        # assuming front sensor is mounted in the center x position (width/2)
        counter = 0  # added for testing
        while True:
            if self.robot_state.is_sim:
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
                curr_ultrasonic_value = self.robot_state.front_ultrasonic.distance()
                if curr_ultrasonic_value < self.robot_state.front_sensor_offset:
                    self.set_phase(Phase.FAULT)
                    return None
            if (self.robot_state.phase == Phase.TRAVERSE) or (self.robot_state.phase == Phase.RETURN) or (self.robot_state.phase == Phase.DOCKING) or (
                    self.robot_state.phase == Phase.AVOID_OBSTACLE):
                if curr_ultrasonic_value < self.robot_state.detect_obstacle_range:
                    # Note: didn't check whether we can reach goal before contacting obstacle because obstacle
                    # detection does not detect angle, so obstacle could be calculated to be falsely farther away than
                    # the goal. Not optimal because in cases, robot will execute boundary following when it can reach
                    # goal
                    self.robot_state.dist_to_goal = self.calculate_dist(self.robot_state.goal_location, self.robot_state.state)
                    self.robot_state.avoid_obstacle = True
                    if self.robot_state.is_sim:
                        with open(ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv', 'a') as fd:
                            fd.write("Avoid" + '\n')
                else:
                    self.robot_state.avoid_obstacle = False
                    if self.robot_state.is_sim:
                        with open(ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv', 'a') as fd:
                            fd.write("Not Avoid" + '\n')
            if curr_ultrasonic_value < 0:
                self.set_phase(Phase.FAULT)  # value should not go below 0; sensor is broken
                if self.robot_state.is_sim:
                    with open(ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv', 'a') as fd:
                        fd.write("Fault" + '\n')

        # time.sleep(10)  # don't hog the cpu

    def execute_avoid_obstacle(self, dist_to_goal, database):
        """ Execute obstacle avoidance
            Args:
                dist_to_goal (Double): The distance from the robot to the goal at the start of the phase
            Returns:
                prev_phase (Phase)
        """
        # TODO: ADD NOISE MARGIN: let t0 be the time when the robot first left the init threshold.
        #  At t0+1, noise can make it such that the robot re-entered the threshold.
        #  Add margin to threshold initially and disable margin once the robot first left threshold.
        init_x = self.robot_state.x_pos
        init_y = self.robot_state.y_pos
        # Note: init_threshold is arbitrary, set later. sometimes location reading will be inaccurate or not frequent
        #  enough to read that we've arrived back to the init x y pos. make it small enough that robot actually leaves
        #  init_threshold at some time during boundary following or add timeout to branch making gate = True
        has_left_init_thresh = False
        did_dist_to_goal_decreased = False
        has_traversed_boundary = False
        curr_dist_to_goal = self.calculate_dist(self.robot_state.goal_location, (self.robot_state.x_pos, self.robot_state.y_pos))
        while True:
            if self.robot_state.phase == Phase.fault:  # fault has priority over obstacle avoidance
                return None
            else:
                dist_from_init = self.calculate_dist((self.robot_state.x_pos, self.robot_state.y_pos), (init_x, init_y))
                if dist_from_init > (self.robot_state.init_threshold + self.robot_state.noise_margin):
                    has_left_init_thresh = True
                if curr_dist_to_goal < self.robot_state.goal_threshold:  # exits obstacle avoidance if robot close to goal
                    self.set_phase(self.robot_state.prev_phase)
                    return None
                elif has_traversed_boundary and has_left_init_thresh:
                    self.set_phase(Phase.FAULT)  # cannot reach goal
                    return None
                elif did_dist_to_goal_decreased:
                    # don't include condition on whether there is a new obstacle bc it will be caught and
                    #  current algorithm will continue traversing current boundary (instead of traversing new obstacle)
                    #  not optimal because frequently will have to go back to obstacle avoidance
                    heading_threshold = 1
                    target_heading = math.atan2(self.robot_state.goal_location[1] - self.robot_state.y_pos, self.robot_state.goal_location[0] - self.robot_state.x_pos)
                    self.turn_to_target_heading(target_heading, heading_threshold, database)
                    curr_ultrasonic_value = self.robot_state.front_ultrasonic.distance()
                    if curr_ultrasonic_value < self.robot_state.detect_obstacle_range:
                        return self.execute_avoid_obstacle(curr_dist_to_goal, database)
                    else:
                        self.set_phase(self.robot_state.prev_phase)
                    return None
                    # TODO: check position of goal and if obstacle in way of goal (using side sensors) then keep doing
                    #  boundary following except with new init_x, init_y, gate,

                else:
                    self.execute_boundary_following(0)  # add code directly here
                    # update conditions
                    curr_dist_to_goal = self.calculate_dist(self.robot_state.goal_location, (self.robot_state.x_pos, self.robot_state.y_pos))
                    did_dist_to_goal_decreased = (curr_dist_to_goal < dist_to_goal)
                    has_traversed_boundary = dist_from_init < self.robot_state.init_threshold
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
        front_dist = self.robot_state.rf_ultrasonic.distance()
        back_dist = self.robot_state.rb_ultrasonic.distance()
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
            # make sure to take into account width and length of robot and ultrasonic sensor value on both sides
            # make sure to take into account situation when the gap you are turning into is smaller than the robot width
        # add cases for obstacles when boundary following
            # gap in wall but robot cannot fit
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
