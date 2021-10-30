import numpy as np
import math
from engine.kinematics import integrate_odom, feedback_lin, limit_cmds, meters_to_gps
from engine.pid_controller import PID
from enum import Enum

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


class Control_Mode(Enum):
    """
    An enumeration of different control modes
    """
    LAWNMOVER = 1
    ROOMBA = 2
    MANUAL = 3


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

    def __init__(self, x_pos, y_pos, heading, epsilon, max_v, radius, is_sim=True, position_kp=1, position_ki=0,
                 position_kd=0, position_noise=0, heading_kp=1, heading_ki=0, heading_kd=0, heading_noise=0,
                 init_phase=1, time_step=0.1, control_mode=1):
        """
        Arguments:
            x_pos: the x position of the robot, where (0,0) is the bottom left corner of the grid with which
                the Robot's related Mission was initialized
            y_pos: the y position of the robot
            heading: the theta of the robot in radians, where North on the grid is equal to 0.
            epsilon: dictates the turning radius of the robot. Lower epsilon results in tighter turning radius.
            max_v: the maximum velocity of the robot
            radius: the radius of the robot
            is_sim: False if the physical robot is being used, True otherwise
            position_kp: the proportional factor of the position PID
            position_ki: the integral factor of the position PID
            position_kd: the derivative factor of the position PID
            position_noise: the flat amount of noise added to the robot's phase on each localization step
            init_phase: the phase which the robot begins at
            heading_kp: the proportional factor of the heading PID
            heading_ki: the integral factor of the heading PID
            heading_kd: the derivative factor of the heading PID
            heading_noise: ?
            init_phase: the phase which the robot begins at
            time_step: the amount of time that passes between each feedback loop cycle, should only be used if is_sim
                is True
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
        self.control_mode = Control_Mode(control_mode)

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

    def travel(self, dist, turn_angle):
        # Moves the robot with both linear and angular velocity
        self.state = np.round(integrate_odom(self.state, dist, turn_angle), 3)

        # if it is a simulation,
        if self.is_sim:
            self.truthpose = np.append(self.truthpose, np.transpose(self.state), 0)

    def move_forward(self, dist, dt=1):
        # Moves robot forward by distance dist
        # dist is in meters
        new_x = self.state[0] + dist * math.cos(self.state[2]) * dt
        new_y = self.state[0] + dist * math.sin(self.state[2]) * dt
        self.state[0] = np.round(new_x, 3)
        self.state[1] = np.round(new_y, 3)

        if self.is_sim:
            self.truthpose = np.append(self.truthpose, np.transpose(self.state), 0)

    def move_to_target_node(self, target, allowed_dist_error):
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
            self.state[0] = np.random.normal(self.state[0], self.position_noise)
            self.state[1] = np.random.normal(self.state[1], self.position_noise)

            x_error = target[0] - self.state[0]
            y_error = target[1] - self.state[1]

            x_vel = self.loc_pid_x.update(x_error)
            y_vel = self.loc_pid_y.update(y_error)

            # the x_vel and y_vel we pass into feedback_lin should be global. Are they?
            cmd_v, cmd_w = feedback_lin(predicted_state, x_vel, y_vel, self.epsilon)

            # clamping of velocities?
            (limited_cmd_v, limited_cmd_w) = limit_cmds(cmd_v, cmd_w, self.max_velocity, self.radius)

            self.travel(self.time_step * limited_cmd_v, self.time_step * limited_cmd_w)
            # sleep in real robot.

            # Get state after movement:
            predicted_state = self.state  # this will come from Kalman Filter
            # location error (in meters)
            distance_away = math.hypot(float(predicted_state[0]) - target[0],
                                       float(predicted_state[1]) - target[1])

    def turn_to_target_heading(self, target_heading, allowed_heading_error):
        """
        Turns robot in-place to target heading + or - allowed_heading_error, utilizing heading PID.

        Arguments:
            target_heading: the heading in radians the robot should approach at the end of in-place rotation.
            allowed_heading_error: the maximum error in radians a robot can have to target heading while turning in
                place.
        """

        predicted_state = self.state  # this will come from Kalman Filter

        abs_heading_error = abs(target_heading-float(predicted_state[2]))

        while abs_heading_error > allowed_heading_error:
            self.state[2] = np.random.normal(self.state[2], self.heading_noise)
            theta_error = target_heading - self.state[2]
            w = self.head_pid.update(theta_error)  # angular velocity
            _, limited_cmd_w = limit_cmds(0, w, self.max_velocity, self.radius)

            self.travel(0, self.time_step * limited_cmd_w)
            # sleep in real robot

            # Get state after movement:
            predicted_state = self.state  # this will come from Kalman Filter
            abs_heading_error = abs(target_heading - float(predicted_state[2]))

    def turn(self, turn_angle, dt=1):
        """
        Hardcoded in-place rotation for testing purposes. Does not use heading PID. Avoid using in physical robot.
        """
        # Turns robot, where turn_angle is given in radians
        clamp_angle = (self.state[2] + (turn_angle * dt)) % (2 * math.pi)
        self.state[2] = np.round(clamp_angle, 3)
        if self.is_sim:
            self.truthpose = np.append(self.truthpose, np.transpose(self.state), 0)

    def get_state(self):
        return self.state

    def print_current_state(self):
        # Prints current robot state
        print('pos: ' + str(self.state[0:2]))
        print('heading: ' + str(self.state[2]))

    def execute_setup(self):
        pass

    def execute_traversal(self, unvisited_waypoints, allowed_dist_error, grid, control_mode, time_limit):
        match control_mode:
            case Control_Mode.LAWNMOVER:
                self.lawn_mover(unvisited_waypoints, allowed_dist_error)
            case Control_Mode.ROOMBA:
                self.roomba_mover(grid, time_limit)

    def lawn_mover(self, unvisited_waypoints, allowed_dist_error):        
        while unvisited_waypoints:
            curr_waypoint = unvisited_waypoints[0].get_m_coords()
            self.move_to_target_node(curr_waypoint, allowed_dist_error)  # TODO: add obstacle avoidance support
            unvisited_waypoints.popleft()

        self.phase = Phase.RETURN
        return unvisited_waypoints

    def roomba_mover(self, grid, time_limit):
        # move the robot in a roomba-like manner
        move_dist = 3  # how much the robot moves in meter
        turn_dist = .5233  # how much the robot turns in rads
        dt = 0
        exit_boolean = False # battery_limit, time_limit, battery_capacity is full

        while not exit_boolean:
            curr_x = self.state[0]
            curr_y = self.state[1]
            # lowerleft (lower bound) of the grid is 0,0. upperright bound would be difference between max and min
            roomba_out_of_x_bound = curr_x < 0 or curr_x > (grid.long_max-grid.long_min)
            roomba_out_of_y_bound = curr_y < 0 or curr_y > (grid.lat_max-grid.lat_min)

            if roomba_out_of_x_bound or roomba_out_of_y_bound:
                self.move_forward(-move_dist)
                self.turn(turn_dist)
            else:
                self.move_forward(move_dist)
            dt += dt
            exit_boolean = (dt > time_limit)
        self.phase = Phase.COMPLETE
        return None

    def execute_avoid_obstacle(self):
        pass

    def execute_return(self, base_loc, base_angle, allowed_docking_pos_error, allowed_heading_error):
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

        self.move_to_target_node(target_loc, allowed_docking_pos_error)  # TODO: add obstacle avoidance support

        # Face robot towards base station
        target_heading = base_angle + math.pi
        self.turn_to_target_heading(target_heading, allowed_heading_error)

        # RETURN phase complete:
        self.phase = Phase.DOCKING

    def execute_docking(self):
        self.phase = Phase.COMPLETE  # temporary for simulation purposes
