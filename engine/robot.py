import threading
import time
import numpy as np
import math

from engine.phase import Phase
from engine.ekf import LocalizationEKF
from engine.pid_controller import PID
from constants.definitions import *
from engine.kinematics import integrate_odom, feedback_lin, limit_cmds, get_vincenty_x, get_vincenty_y
from csv_files.csv_util import write_state_to_csv, write_phase_to_csv
from electrical.ultrasonic_sensor import Ultrasonic


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
        """
            Arguments:
            robot_state: an instance encapsulating conditions, measurements, etc. (i.e. all data) about this robot  
        """
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
        if self.robot_state.should_store_data:
            write_phase_to_csv(self.robot_state.phase)

    def update_ekf_step(self):
        zone = ENGINEERING_QUAD  # Used for GPS visualization, make this not hard-coded
        # self.robot_state.ekf.update_step(self.robot_state.ekf.mu, self.robot_state.ekf.sigma, sensor_module.get_measurement(self.robot_state.init_gps))
        self.robot_state.gps_data = (self.robot_state.gps.get_gps()[
                                     "long"], self.robot_state.gps.get_gps()["lat"])
        self.robot_state.imu_data = self.robot_state.imu.get_gps()
        x, y = get_vincenty_x(
            self.robot_state.init_gps, self.robot_state.gps_data), get_vincenty_y(self.robot_state.init_gps, self.robot_state.gps_data)
        heading = math.degrees(math.atan2(
            self.robot_state.imu_data["mag"]["y"], self.robot_state.imu_data["mag"]["x"]))

        measurements = np.array([[x], [y], [heading]])

        self.robot_state.ekf.update_step(
            self.robot_state.ekf.mu, self.robot_state.ekf.sigma, measurements)
        new_x = self.robot_state.ekf.mu[0][0]
        new_y = self.robot_state.ekf.mu[1][0]
        new_heading = self.robot_state.ekf.mu[2][0]
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
            self.robot_state.state[2] = math.degrees(
                math.atan2(imu_data["mag"][1], imu_data["mag"][0]))

    def move_forward(self, dist):
        # Moves robot forward by distance dist
        # dist is in meters
        new_x = self.robot_state.state[0] + dist * math.cos(
            self.robot_state.state[2]) * self.robot_state.time_step
        new_y = self.robot_state.state[1] + dist * math.sin(
            self.robot_state.state[2]) * self.robot_state.time_step
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

            if self.robot_state.is_sim and self.robot_state.should_store_data:
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

        predicted_state = self.robot_state.state  # this will come from Kalman Filter

        abs_heading_error = abs(target_heading - float(predicted_state[2]))

        while abs_heading_error > allowed_heading_error:
            if self.robot_state.is_sim:
                self.robot_state.state[2] = np.random.normal(
                    self.robot_state.state[2], self.robot_state.heading_noise)
            else:
                self.robot_state.state = self.update_ekf_step()
            theta_error = target_heading - self.robot_state.state[2]
            w = self.head_pid.update(theta_error)  # angular velocity
            _, limited_cmd_w = limit_cmds(
                0, w, self.robot_state.max_velocity, self.robot_state.radius)

            if self.robot_state.is_sim:
                self.travel(0, self.robot_state.time_step * limited_cmd_w)
            else:
                self.robot_state.motor_controller.spin_motors(limited_cmd_w, 0)
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
        self.robot_state.init_gps = (
            gps.get_gps()["long"], gps.get_gps()["lat"])
        self.robot_state.imu_data = imu.get_gps()
        x_init, y_init = (0, 0)
        heading_init = math.degrees(math.atan2(
            self.robot_state.imu_data["mag"]["y"], self.robot_state.imu_data["mag"]["x"]))

        # mu is meters from start position (bottom left position facing up)
        mu = np.array([[x_init], [y_init], [heading_init]])

        # confidence of mu, set it to high initially b/c not confident, algo brings it down
        sigma = np.array([[10, 0, 0], [0, 10, 0], [0, 0, 10]])
        self.robot_state.ekf = LocalizationEKF(mu, sigma)
        self.robot_state.gps = gps
        self.robot_state.imu = imu
        self.robot_state.motor_controller = motor_controller
        if (radio_session.connected and gps_setup and imu_setup):
            obstacle_avoidance = threading.Thread(
                target=self.track_obstacle, daemon=True)
            obstacle_avoidance.start()  # spawn thread to monitor obstacles
            self.set_phase(Phase.TRAVERSE)

    def execute_traversal(self, unvisited_waypoints, allowed_dist_error, base_station_loc, control_mode, time_limit,
                          roomba_radius, database):
        self.robot_state.control_mode = control_mode
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
            # TODO: THIS ISNT CORRECT: NEED TO CHECK IF AVOID_OBSTACLE IN move_to_target_node or can also make PID traversal a separate thread and stop the thread when obstacle detected

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
            front_ultrasonic = Ultrasonic(0)
            # sensor should not detect something in the robot
            if front_ultrasonic.distance < self.robot_state.front_sensor_offset:
                self.set_phase(Phase.FAULT)
                return None
            curr_x = self.robot_state.state[0]
            curr_y = self.robot_state.state[1]
            new_x = curr_x + self.robot_state.move_dist * \
                math.cos(self.robot_state.state[2]
                         ) * self.robot_state.time_step
            new_y = curr_y + self.robot_state.move_dist * \
                math.sin(self.robot_state.state[2]
                         ) * self.robot_state.time_step
            next_radius = self.calculate_dist(base_station_loc, (new_x, new_y))
            # if moving will cause the robot to move through the obstacle
            is_next_timestep_blocked = next_radius < self.robot_state.detect_obstacle_range
            # sensor should not detect something in the robot
            if (next_radius > roomba_radius) or (self.robot_state.is_roomba_obstacle and is_next_timestep_blocked):
                # this needs to be synchronous/PID'ed, otherwise, turn might be called while robot moving forward
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
                    # TODO: determine what vel to run this at
                    self.robot_state.motor_controller.motors(0, 0)
            dt += 1
            exit_boolean = (dt > time_limit)
        self.set_phase(Phase.COMPLETE)  # TODO: CHANGE the next phase to return
        return None

    def set_phase(self, new_phase):
        self.robot_state.phase = new_phase
        if self.robot_state.should_store_data:
            write_phase_to_csv(self.robot_state.phase)

    def track_obstacle(self):
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
            if self.robot_state.is_sim:
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
                front_ultrasonic = Ultrasonic(0)
                curr_ultrasonic_value = front_ultrasonic.distance()
                if curr_ultrasonic_value < self.robot_state.front_sensor_offset:
                    self.set_phase(Phase.FAULT)
                    return None
            if self.robot_state.control_mode == 4:  # roomba mode
                self.robot_state.is_roomba_obstacle = True
            elif (self.robot_state.phase == Phase.TRAVERSE) or (self.robot_state.phase == Phase.RETURN) or (self.robot_state.phase == Phase.DOCKING) or (
                    self.robot_state.phase == Phase.AVOID_OBSTACLE):
                if curr_ultrasonic_value < self.robot_state.detect_obstacle_range:
                    # Note: didn't check whether we can reach goal before contacting obstacle because obstacle
                    # detection does not detect angle, so obstacle could be calculated to be falsely farther away than
                    # the goal. Not optimal because in cases, robot will execute boundary following when it can reach
                    # goal
                    self.set_phase(Phase.AVOID_OBSTACLE)
                    if self.robot_state.is_sim:
                        with open(ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv', 'a') as fd:
                            fd.write("Avoid" + '\n')
                else:
                    if self.robot_state.is_sim:
                        with open(ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv', 'a') as fd:
                            fd.write("Not Avoid" + '\n')
            if curr_ultrasonic_value < 0:
                # value should not go below 0; sensor is broken
                self.set_phase(Phase.FAULT)
                if self.robot_state.is_sim:
                    with open(ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv', 'a') as fd:
                        fd.write("Fault" + '\n')
            if not self.robot_state.is_sim:
                time.sleep(10)  # don't hog the cpu

    def is_on_line(self, p1, p2, tolerance):
        """ Checks if the robot is on the line defined by the two points p1 and p2
            Args:
                p1 (Tuple): An (x, y) point on the line
                p2 (Tuple): Another (x, y) point on the line
                tolerance (Float): How far two thetas can be (from the x-axis) to be considered on the same line
        """
        # convert to polar coord bc euclidean coord has potentional divide by 0 err when calculating slope of vertical line
        line_theta = math.atan2((p2[1] - p1[1]), (p2[0] - p1[0]))
        # cant get multiple thetas given one point without them not being on the same line
        new_theta = math.atan2(
            (p2[1] - self.robot_state.state[1]), (p2[0] - self.robot_state.state[0]))
        difference = new_theta - line_theta
        # want to detect same line even if rotated 180
        if abs(difference) == math.pi:
            difference = 0
        return abs(difference) <= tolerance

    def execute_avoid_obstacle(self, on_line_tolerance, init_threshold=3., goal_threshold=3., threshold_to_recalculate_on_line=3.):
        """ Execute bug 2 (straight line) obstacle avoidance algorithm
            Args:
                on_line_tolerance (float): threshold in meters for checking theta matches for the lines
                init_threshold (float): Radius in meters from initial position that will detect robot is back in initial position
                    This parameter needs to be tuned.
                    The reasoning behind this parameter is that sometimes location reading will be inaccurate or not frequent read
                    so the sensor thinks we're still at the init x y pos. We should make this small enough that robot actually leaves
                    init_threshold at some time during boundary following or add a timeout
                goal_threshold (float): Threshold in meters from goal that will be detected as reaching goal in obstacle avoidance.
                    Goal is where the robot wanted to go when there isn't an obstacle
                threshold_to_recalculate_on_line (float): threshold in meters that we dont want to recalculate whether the robot is on the line to the goal
                    This parameter needs to be tuned
                    Reasoning for this: without this condition, the robot could move closer to the goal (on a slant, not directly towards the goal)
                    but still be on the line, making the robot keep exiting obstacle avoidance when its effectively in the same position as before

            Returns:
                prev_phase (Phase)
        """
        init_x = self.robot_state.state[0]
        init_y = self.robot_state.state[1]
        init_pos = (init_x, init_y)

        # last position of the robot when it's still on the (line from its initial position to its goal positions)
        last_pos_on_line = init_pos
        # the robot's initial location is x,y and the init_threshold is d.
        # If the robot's new location x',y' is more than d distance from init_threshold,
        # we set has_left_init_threshold to true. Used to determine if the robot has left its initial location
        has_left_init_thresh = False
        has_traversed_boundary = False
        init_dist_to_goal = self.calculate_dist(
            self.robot_state.goal_location, curr_pos)
        while True:
            # fault has priority over obstacle avoidance
            if self.robot_state.phase == Phase.FAULT:
                return None
            curr_pos = (self.robot_state.state[0], self.robot_state.state[1])
            curr_dist_to_goal = self.calculate_dist(
                self.robot_state.goal_location, curr_pos)
            dist_from_init = self.calculate_dist(curr_pos, init_pos)
            is_on_line = self.is_on_line(
                self.robot_state.goal_location, curr_pos, on_line_tolerance)
            if dist_from_init > init_threshold:
                has_left_init_thresh = True
            # exits obstacle avoidance if robot close to goal
            if curr_dist_to_goal < goal_threshold:
                self.set_phase(self.robot_state.prev_phase)
                return None
            elif has_traversed_boundary:
                self.set_phase(Phase.FAULT)  # cannot reach goal
                return None
            # bug 2
            elif is_on_line and (curr_dist_to_goal < init_dist_to_goal):
                if self.calculate_dist(curr_pos, last_pos_on_line) > threshold_to_recalculate_on_line:
                    self.set_phase(self.robot_state.prev_phase)
                    return None
            else:
                self.execute_boundary_following()
            if has_left_init_thresh:
                # we dont want the program to think that the boundary is untraversable
                #   when the robot is still in the threshold from where it started obstacle avoidance
                has_traversed_boundary = dist_from_init < init_threshold
            time.sleep(10)  # don't hog the cpu

    def execute_boundary_following(self):
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

        For now, we assume the side ultrasonic sensor values are from the actual sensors, so changes needed for is_sim
        '''
        front_ultrasonic = Ultrasonic(0)
        rf_ultrasonic = Ultrasonic(1)
        rb_ultrasonic = Ultrasonic(2)
        lf_ultrasonic = Ultrasonic(3)
        # robot should be side_margin_sensor meters away from side obstacle, placeholder
        side_margin_sensor = 1
        # robot should be margin_to_front_obstacle meters away from front obstacle, placeholder. Want space to turn
        margin_to_front_obstacle = 5 + \
            max(self.robot_state.width, self.robot_state.length)
        # When boundary following gap in wall but robot cannot fit (check both side and turn 180 and do boundary following again)
        if lf_ultrasonic.distance < side_margin_sensor:
            curr_heading = self.robot_state.state[2]
            if curr_heading > math.pi:
                desired_heading = curr_heading - math.pi
            else:
                desired_heading = curr_heading + math.pi
            # error is arbitrary
            heading_err = 10
            self.turn_to_target_heading(desired_heading, heading_err)
        # if there is an obstacle in front of the robot, turn until there isn't
        elif front_ultrasonic.distance < margin_to_front_obstacle:
            self.robot_state.motor_controller.spin_motors(
                0, self.robot_state.turn_angle)
        # if there is no obstacle in front of the robot but it's still in avoid obstacle because it's following the boundary
        else:
            rf_dist = rf_ultrasonic.distance()
            rb_dist = rb_ultrasonic.distance()
            # if the robot is approximately parallel to the robot, go forward
            if abs(rf_dist - rb_dist) < side_margin_sensor:
                self.robot_state.motor_controller.spin_motors(
                    self.robot_state.move_dist, 0)
            # otherwise, turn until the robot is parallel
            else:
                direction = (rf_dist - rb_dist)/abs(rf_dist - rb_dist)
                self.robot_state.motor_controller.spin_motors(
                    0, direction * self.robot_state.turn_angle)
            # NOTE: TEST WHETHER THIS ACTUALLY WORKS WHEN TURNING:
            # IF NOT, MIGHT HAVE TO ADD LOCKS TO FORCE THE ROBOT TO TURN UNTIL IT IS PARALLEL TO ROBOT
            # THIS INVOLVES A LOT OF SEQUENCE OF MOVES, SO IT MIGHT INVOLVE A LOT OF SEQUENTIAL UNLOCKING
            # for example, if robot starts off parallel and it turns until parallel again,
            # lock1 unlocks when robot no longer detects obstacle in front,
            # then lock2 unlocks if lock1 unlocked and rb ultrasonic < rf ultrasonic.
            # This is assuming rb ultrasonic > rf ultrasonic when turning ccw.
            # otherwise, might have to have a condition where robot not moving and just turning
            # and gets back to the same heading to keep turning
    # TODO: add dp to not traverse to node already traversed/in obstacle

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
