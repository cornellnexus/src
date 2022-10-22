import numpy as np
import math
from engine.kinematics import integrate_odom, feedback_lin, limit_cmds
from engine.pid_controller import PID
from electrical.motor_controller import MotorController
from constants.definitions import *

# import electrical.gps as gps
# import electrical.imu as imu
# import electrical.radio_module as radio_module


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
        state contain's the robot's x position, y position, and heading
    phase = 'collect' when traversing through grid, 'return' when returning to
        base station
    is_sim = False when running code with physical robot, True otherwise
    Preconditions:
    position is list of size 2
    heading is in range [0..359]
    """

    def __init__(self, x_pos, y_pos, heading, epsilon, max_v, radius, width, front_ultrasonic, lf_ultrasonic,
                 lb_ultrasonic, rf_ultrasonic, rb_ultrasonic, is_sim=True, position_kp=1, position_ki=0,
                 position_kd=0, position_noise=0, heading_kp=1, heading_ki=0, heading_kd=0, heading_noise=0,
                 init_phase=1, time_step=1, move_dist=.5, turn_angle=3, plastic_weight=0, goal_location=(0, 0)):
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
            goal_location: location of the target location; added for testing obstacle tracking
        """
        self.state = np.array([[x_pos], [y_pos], [heading]])
        self.truthpose = np.transpose(np.array([[x_pos], [y_pos], [heading]]))
        self.is_sim = is_sim
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
        self.linear_v = 0
        self.angular_v = 0
        self.width = width
        self.avoid_obstacle = False
        self.fault = False
        self.front_ultrasonic = front_ultrasonic
        self.lf_ultrasonic = lf_ultrasonic
        self.lb_ultrasonic = lb_ultrasonic
        self.rf_ultrasonic = rf_ultrasonic
        self.rb_ultrasonic = rb_ultrasonic
        self.dist_to_goal = 0
        self.prev_phase = self.phase
        self.goal_location = (0, 0)
        self.max_sensor_range = 600
        self.front_sensor_offset = 0  # replace this with how far offset the sensor is to the front of the robot
        self.sensor_measuring_angle = 75
        self.width_margin = 1  # replace this with actual margin
        self.threshold_distance = ((self.width + self.width_margin) / 2) / math.cos(
            math.radians((180 - self.sensor_measuring_angle) / 2)) + self.front_sensor_offset
        self.detect_obstacle_range = min(self.threshold_distance, self.max_sensor_range)  # set ultrasonic detection range

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
        with open(CSV_PATH + '/phases.csv', 'a') as fd:
            fd.write(str(self.phase) + '\n')

    def travel(self, dist, turn_angle):
        # Moves the robot with both linear and angular velocity
        self.state = np.round(integrate_odom(self.state, dist, turn_angle), 3)
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
            allowed_dist_error: the maximum distance in meters that the robot can be from a node for the robot to
                have "visited" that node
        """
        predicted_state = self.state  # this will come from Kalman Filter

        # location error (in meters)
        distance_away = math.hypot(float(predicted_state[0]) - target[0],
                                   float(predicted_state[1]) - target[1])

        while distance_away > allowed_dist_error:
            self.state[0] = np.random.normal(
                self.state[0], self.position_noise)
            self.state[1] = np.random.normal(
                self.state[1], self.position_noise)

            x_error = target[0] - self.state[0]
            y_error = target[1] - self.state[1]

            x_vel = self.loc_pid_x.update(x_error)
            y_vel = self.loc_pid_y.update(y_error)

            cmd_v, cmd_w = feedback_lin(
                predicted_state, x_vel, y_vel, self.epsilon)

            # clamping of velocities:
            (limited_cmd_v, limited_cmd_w) = limit_cmds(
                cmd_v, cmd_w, self.max_velocity, self.radius)

            self.travel(self.time_step * limited_cmd_v,
                        self.time_step * limited_cmd_w)

            self.linear_v = limited_cmd_v[0]
            self.angular_v = limited_cmd_w[0]

            # sleep in real robot.

            # write robot location and mag heading in csv (for gui to display)
            with open(CSV_PATH + '/datastore.csv', 'a') as fd:
                fd.write(
                    str(self.state[0])[1:-1] + ',' + str(self.state[1])[1:-1] + ',' + str(self.state[2])[1:-1] + '\n')
            time.sleep(0.001)

            # Get state after movement:
            predicted_state = self.state  # this will come from Kalman Filter
            # TODO: Do we want to update self.state with this new predicted state????
            database.update_data(
                "state", self.state[0], self.state[1], self.state[2])

            # location error (in meters)
            distance_away = math.hypot(float(predicted_state[0]) - target[0],
                                       float(predicted_state[1]) - target[1])

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
            self.state[2] = np.random.normal(self.state[2], self.heading_noise)
            theta_error = target_heading - self.state[2]
            w = self.head_pid.update(theta_error)  # angular velocity
            _, limited_cmd_w = limit_cmds(0, w, self.max_velocity, self.radius)

            self.travel(0, self.time_step * limited_cmd_w)
            # sleep in real robot

            # Get state after movement:
            predicted_state = self.state  # this will come from Kalman Filter
            # TODO: Do we want to update self.state with this new predicted state????
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
        motor_controller.setup()
        if (radio_session.connected and gps_setup and imu_setup):
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
        exit_boolean = False  # TODO: battery_limit, time_limit, tank_capacity is full
        while not exit_boolean:

            curr_x = self.state[0]
            curr_y = self.state[1]
            new_x = curr_x + self.move_dist * math.cos(self.state[2]) * self.time_step
            new_y = curr_y + self.move_dist * math.sin(self.state[2]) * self.time_step
            next_radius = math.sqrt(
                abs(new_x - base_station_loc[0]) ** 2 + abs(new_y - base_station_loc[1]) ** 2)
            obstacle_detected = self.front_ultrasonic.distance() > self.detect_obstacle_range
            # if moving will cause the robot to move through the obstacle
            next_after_obstacle = next_radius < self.detect_obstacle_range
            if (next_radius > roomba_radius) or (obstacle_detected and next_after_obstacle):
                self.move_forward(-self.move_dist)
                self.turn(self.turn_angle)
            else:
                self.move_forward(self.move_dist)
            dt += 1
            exit_boolean = (dt > time_limit)
            # if self.avoid_obstacle:
            #     self.set_phase(Phase.AVOID_OBSTACLE)
            #     self.prev_phase = Phase.TRAVERSE
            #     return None
        self.set_phase(Phase.COMPLETE)  # TODO: CHANGE the next phase to return
        return None

    def set_phase(self, new_phase):
        self.phase = new_phase

        with open(CSV_PATH + '/phases.csv', 'a') as fd:
            fd.write(str(self.phase) + '\n')

    def track_obstacle(self):
        pass

    def execute_avoid_obstacle(self, dist_to_goal, prev_phase):
        pass

    def execute_boundary_following(self, min_dist):
        front_dist = self.rf_ultrasonic.distance()
        back_dist = self.rb_ultrasonic.distance()
        margin = 1  # Will change
        forward_dist = 0.01  # Will change
        turn_angle = math.pi / 90  # Will change

        if math.abs(front_dist - back_dist) < margin:
            self.move_forward(forward_dist)
        elif front_dist > back_dist:
            self.turn(turn_angle)
        else:
            self.turn(-1 * turn_angle)

        '''
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