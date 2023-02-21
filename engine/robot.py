import numpy as np
import math
from engine.kinematics import integrate_odom, feedback_lin, limit_cmds
from engine.pid_controller import PID
from electrical.motor_controller import MotorController
from constants.definitions import CSV_PATH
from engine.robot_state import robot_state


# import electrical.gps as gps 
# import electrical.imu as imu 
# import electrical.radio_module as radio_module

import time
import sys

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

    def __init__(self, robot_state):
        self.loc_pid_x = PID(
            Kp=robot_state.position_kp, Ki=robot_state.position_ki, Kd=robot_state.position_kd, target=0, sample_time=robot_state.time_step,
            output_limits=(None, None)
        )

        self.loc_pid_y = PID(
            Kp=robot_state.position_kp, Ki=robot_state.position_ki, Kd=robot_state.position_kd, target=0, sample_time=robot_state.time_step,
            output_limits=(None, None)
        )

        self.head_pid = PID(
            Kp=robot_state.heading_kp, Ki=robot_state.heading_ki, Kd=robot_state.heading_kd, target=0, sample_time=robot_state.time_step,
            output_limits=(None, None)
        )

        # TODO: wrap in try/except (error when calling execute_setup_test.py)
        # write in csv
        with open(CSV_PATH + '/phases.csv', 'a') as fd:
            fd.write(str(robot_state.phase) + '\n')

    def travel(self, dist, turn_angle):
        # Moves the robot with both linear and angular velocity
        robot_state.state = np.round(integrate_odom(robot_state.state, dist, turn_angle), 3)
        imu_data = self.imu.get_imu()
        robot_state.state[2] = math.degrees(math.atan2(
            imu_data["mag"][1], imu_data["mag"][0]))
        # if it is a simulation,
        if robot_state.is_sim:
            robot_state.truthpose = np.append(
                robot_state.truthpose, np.transpose(robot_state.state), 0)

    def move_forward(self, dist):
        # Moves robot forward by distance dist
        # dist is in meters
        new_x = robot_state.state[0] + dist * math.cos(robot_state.state[2]) * robot_state.time_step
        new_y = robot_state.state[1] + dist * math.sin(robot_state.state[2]) * robot_state.time_step
        robot_state.state[0] = np.round(new_x, 3)
        robot_state.state[1] = np.round(new_y, 3)

        if robot_state.is_sim:
            robot_state.truthpose = np.append(
                robot_state.truthpose, np.transpose(robot_state.state), 0)

    def move_to_target_node(self, target, allowed_dist_error, database):
        """
        Moves robot to target + or - allowed_dist_error

        Arguments:
            target: target coordinates in the form (latitude, longitude)
            allowed_dist_error: the maximum distance in meters that the robot can be from a node for the robot to
                have "visited" that node
        """
        predicted_state = robot_state.state  # this will come from Kalman Filter

        # location error (in meters)
        distance_away = math.hypot(float(predicted_state[0]) - target[0],
                                   float(predicted_state[1]) - target[1])

        while distance_away > allowed_dist_error:
            robot_state.state[0] = np.random.normal(
                robot_state.state[0], robot_state.position_noise)
            robot_state.state[1] = np.random.normal(
                robot_state.state[1], robot_state.position_noise)

            x_error = target[0] - robot_state.state[0]
            y_error = target[1] - robot_state.state[1]

            x_vel = self.loc_pid_x.update(x_error)
            y_vel = self.loc_pid_y.update(y_error)

            cmd_v, cmd_w = feedback_lin(
                predicted_state, x_vel, y_vel, robot_state.epsilon)

            # clamping of velocities:
            (limited_cmd_v, limited_cmd_w) = limit_cmds(
                cmd_v, cmd_w, robot_state.max_velocity, robot_state.radius)

            robot_state.linear_v = limited_cmd_v[0]
            robot_state.angular_v = limited_cmd_w[0]

            self.travel(robot_state.time_step * limited_cmd_v,
                        robot_state.time_step * limited_cmd_w)
            
            # for real robot: 
            #TODO: pass in motor controller or initialize it somewhere within the codebase
            #      to call the spin_motors function
            # self.motor_controller.spin_motors(Robot_State.angular_v, Robot_State.linear_v) 

            # write robot location and mag heading in csv (for gui to display)
            with open(CSV_PATH + '/datastore.csv', 'a') as fd:
                fd.write(
                    str(robot_state.state[0])[1:-1] + ',' + str(robot_state.state[1])[1:-1] + ',' + str(robot_state.state[2])[1:-1] + '\n')
            time.sleep(0.001)

            # Get state after movement:
            predicted_state = robot_state.state  # this will come from Kalman Filter
            # TODO: Do we want to update Robot_State.state with this new predicted state????
            database.update_data(
                "state", robot_state.state[0], robot_state.state[1], robot_state.state[2])

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

        predicted_state = robot_state.state  # this will come from Kalman Filter

        abs_heading_error = abs(target_heading-float(predicted_state[2]))

        while abs_heading_error > allowed_heading_error:
            robot_state.state[2] = np.random.normal(robot_state.state[2], robot_state.heading_noise)
            theta_error = target_heading - robot_state.state[2]
            w = self.head_pid.update(theta_error)  # angular velocity
            _, limited_cmd_w = limit_cmds(0, w, robot_state.max_velocity, robot_state.radius)

            self.travel(0, robot_state.time_step * limited_cmd_w)
            # sleep in real robot

            # Get state after movement:
            predicted_state = robot_state.state  # this will come from Kalman Filter
            # TODO: Do we want to update Robot_State.state with this new predicted state????
            database.update_data(
                "state", robot_state.state[0], robot_state.state[1], robot_state.state[2])

            abs_heading_error = abs(target_heading - float(predicted_state[2]))

    def turn(self, turn_angle):
        """
        Hardcoded in-place rotation for testing purposes. Does not use heading PID. Avoid using in physical robot.
        """
        # Turns robot, where turn_angle is given in radians
        clamp_angle = (robot_state.state[2] + (turn_angle *
                       robot_state.time_step)) % (2 * math.pi)
        robot_state.state[2] = np.round(clamp_angle, 3)
        if robot_state.is_sim:
            robot_state.truthpose = np.append(
                robot_state.truthpose, np.transpose(robot_state.state), 0)

    def get_state(self):
        return robot_state.state

    def print_current_state(self):
        # Prints current robot state
        print('pos: ' + str(robot_state.state[0:2]))
        print('heading: ' + str(robot_state.state[2]))

    def execute_setup(self, radio_session, gps, imu, motor_controller):
        gps_setup = gps.setup() 
        imu_setup = imu.setup()
        radio_session.setup_robot()
        motor_controller.setup()

        if (radio_session.connected and gps_setup and imu_setup): 
            robot_state.phase = Phase.TRAVERSE


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
            self.move_to_target_node(
                curr_waypoint, allowed_dist_error, database)
            unvisited_waypoints.popleft()

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
        dt = 0
        exit_boolean = False  # TODO: battery_limit, time_limit, tank_capacity is full, obstacle avoiding
        while not exit_boolean:

            curr_x = robot_state.state[0]
            curr_y = robot_state.state[1]
            new_x = curr_x + robot_state.move_dist * \
                math.cos(robot_state.state[2]) * robot_state.time_step
            new_y = curr_y + robot_state.move_dist * \
                math.sin(robot_state.state[2]) * robot_state.time_step
            next_radius = math.sqrt(
                abs(new_x-base_station_loc[0])**2 + abs(new_y-base_station_loc[1])**2)
            if next_radius > roomba_radius:
                self.move_forward(-robot_state.move_dist)
                self.turn(robot_state.turn_angle)
            else:
                self.move_forward(robot_state.move_dist)
            dt += 1
            exit_boolean = (dt > time_limit)
        robot_state.phase = Phase.COMPLETE # TODO: CHANGE the next phase to return
        return None

    def set_phase(self, new_phase):
        robot_state.phase = new_phase

        with open(CSV_PATH + '/phases.csv', 'a') as fd:
            fd.write(str(robot_state.phase) + '\n')

    def execute_avoid_obstacle(self):
        # TODO: SET BACK TO ORIGINAL MISSION (TRAVERSE OR RETURN)
        pass

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
            target_loc, allowed_docking_pos_error, database, motor_controller)

        # Face robot towards base station
        target_heading = base_angle + math.pi
        self.turn_to_target_heading(
            target_heading, allowed_heading_error, database)

        # RETURN phase complete:
        self.set_phase(Phase.DOCKING)

    def execute_docking(self):
        # TODO: add traverse to doc at base station in case mission didnt finish
        self.set_phase(Phase.COMPLETE)  # temporary for simulation purposes
