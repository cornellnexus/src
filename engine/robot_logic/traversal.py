import time
import numpy as np
import math
from engine.control_mode import ControlMode
from engine.phase import Phase
from constants.definitions import *
from engine.kinematics import *
from engine.robot_logic.robot_helpers import phase_change, calculate_dist
from csv_files.csv_util import write_state_to_csv

def traversal_logic(robot_state, mission_state, database):
        """
        Function called by our Mission to start the traversal phase. 
        Currently we have two different traversal modes, our standard lawn-mower 
        traversal and our roomba-mode traversal. 
        """
        control_mode = robot_state.control_mode
        if control_mode == ControlMode.LAWNMOWER:  
            robot_state.is_roomba_traversal = False
            return traverse_standard(robot_state, mission_state.waypoints_to_visit,
                                     mission_state.allowed_dist_error, database)
        elif control_mode == ControlMode.ROOMBA:
            robot_state.is_roomba_traversal = True
            return traverse_roomba(robot_state, mission_state.base_station_loc,
                                   mission_state.time_limit, mission_state.roomba_radius)

def traverse_standard(robot_state, unvisited_waypoints, allowed_dist_error, database):
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
        # Removed function move to target heading since it doesn't really make sense
        move_to_target_node(robot_state, curr_waypoint, allowed_dist_error, database)
        unvisited_waypoints.popleft()
        if robot_state.phase == Phase.AVOID_OBSTACLE: # TODO: Temp fix until goal loc reset
            robot_state.phase == Phase.AVOID_OBSTACLE
            phase_change(robot_state)
            robot_state.goal_location = curr_waypoint
            robot_state.prev_phase = Phase.TRAVERSE
            return unvisited_waypoints
        # TODO: THIS ISNT CORRECT: NEED TO CHECK IF AVOID_OBSTACLE IN move_to_target_node or can also make PID traversal a separate thread and stop the thread when obstacle detected

    robot_state.phase = Phase.RETURN
    phase_change(robot_state)
    return robot_state, unvisited_waypoints

def move_to_target_node(robot_state, target, allowed_dist_error, database):
    """
    Moves robot to target + or - allowed_dist_error
    Arguments:
        target: target coordinates in the form (latitude, longitude)
        allowed_dist_error: the maximum distance in meters that the robot 
        can be from a node for the robot to have "visited" that node
        database: 
    """
    predicted_state = robot_state.state  # this will come from Kalman Filter

    # location error (in meters)
    distance_away = calculate_dist(target, predicted_state)
    robot_state.loc_pid_x.reset_integral()
    robot_state.loc_pid_y.reset_integral()
    while distance_away > allowed_dist_error:
        # Error in terms of latitude and longitude, NOT meters
        x_coords_error = target[0] - robot_state.state[0]
        y_coords_error = target[1] - robot_state.state[1]

        x_vel = robot_state.loc_pid_x.update(x_coords_error)
        y_vel = robot_state.loc_pid_y.update(y_coords_error)

        cmd_v, cmd_w = feedback_lin(
            predicted_state, x_vel, y_vel, robot_state.epsilon)

        # clamping of velocities:
        (limited_cmd_v, limited_cmd_w) = limit_cmds(
            cmd_v, cmd_w, robot_state.max_velocity, robot_state.radius)
        # self.linear_v = limited_cmd_v[0]
        # self.angular_v = limited_cmd_w[0]

        robot_state.linear_v = limited_cmd_v[0]
        robot_state.angular_v = limited_cmd_w[0]
        travel(robot_state, robot_state.time_step * limited_cmd_v[0], robot_state.time_step * limited_cmd_w[0])
        if not robot_state.is_sim:
            robot_state.motor_controller.spin_motors(
                limited_cmd_w[0], limited_cmd_v[0])
            time.sleep(10)

        # Get state after movement:
        predicted_state = robot_state.state  # this will come from Kalman Filter

        if robot_state.is_sim and robot_state.should_store_data:
            # TODO: Update to use databse information
            # FOR GUI: writing robot location and mag heading in CSV
            write_state_to_csv(predicted_state)
            time.sleep(0.001)  # Delays calculation for GUI map

        # FOR DATABASE: updating our database with new predicted state
        # TODO: can the code above be simplified / use the database instead?
        database.update_data(
            "state", predicted_state[0], predicted_state[1], predicted_state[2])

        # location error (in meters)
        distance_away = calculate_dist(target, predicted_state)

def travel(robot_state, delta_d, delta_phi):
    """
    Moves the robot delta_d dist and delta_phi angle
    Arguments:
        delta_d (float): target meters to travel 
        delta_phi (float): target radians to turn
    """
    # if it is a simulation, we update the robot state directly
    if robot_state.is_sim:
        robot_state.state = np.round(integrate_odom(robot_state.state, delta_d, delta_phi), 3)
        robot_state.truthpose = np.append(robot_state.truthpose, np.transpose(robot_state.state), 0)
    # otherwise, we update the robot state using EKF
    else:
        update_state(robot_state, delta_d/robot_state.time_step, delta_phi/robot_state.time_step)
        
def update_state(robot_state, velocity, omega):
    """
    Updates the state of the robot given the linear velocity (velocity) and angular velocity (omega) of the robot
    Args:
        velocity (Float): linear velocity of robot
        omega (Float): angular velocity of robot
    Returns:
        new_state/measurement (Tuple): new x, y, and heading of robot
    """
    robot_state.gps_data = (robot_state.gps.get_gps()["long"], robot_state.gps.get_gps()["lat"])
    
    robot_state.imu_data = robot_state.imu.get_gps()

    x = get_vincenty_x(robot_state.init_gps, robot_state.gps_data)
    y = get_vincenty_y(robot_state.init_gps, robot_state.gps_data)

    heading = math.degrees(math.atan2(robot_state.imu_data["mag"]["y"], robot_state.imu_data["mag"]["x"]))

    measurements = np.array([[x], [y], [heading]])
    if robot_state.using_ekf:
        mu, sigma = robot_state.ekf.predict_step(velocity, omega)
        robot_state.ekf.update_step(mu, sigma, measurements)
        new_x = robot_state.ekf.mu[0][0]
        new_y = robot_state.ekf.mu[1][0]
        new_heading = robot_state.ekf.mu[2][0]
        new_state = np.array([[new_x], [new_y], [new_heading]])
        return new_state
    return measurements

def traverse_roomba(robot_state, base_station_loc, time_limit, roomba_radius):
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
    from electrical.ultrasonic_sensor import Ultrasonic
    front_ultrasonic = Ultrasonic(0)
    while not exit_boolean:
        # sensor should not detect something in the robot
        if front_ultrasonic.distance() < robot_state.front_sensor_offset:
            robot_state.phase = Phase.FAULT
            phase_change(robot_state)
            return None
        curr_x = robot_state.state[0]
        curr_y = robot_state.state[1]
        curr_head = robot_state.state[2]
        [new_x, new_y, new_theta] = robot_state.ekf.get_predicted_state(
            [curr_x, curr_y, curr_head], [robot_state.move_dist * robot_state.time_step, 0])
        next_radius = calculate_dist(base_station_loc, (new_x, new_y))
        # if moving will cause the robot to move through the obstacle
        is_next_timestep_blocked = next_radius < robot_state.detect_obstacle_range
        # sensor should not detect something in the robot
        if (next_radius > roomba_radius) or (robot_state.is_roomba_obstacle and is_next_timestep_blocked):
            # this needs to be synchronous/PID'ed, otherwise, turn might be called while robot moving forward
            if robot_state.is_sim:
                # for some reason I don't think this should work. This needs to be blocking: wait for the robot to finish going backward before turning
                travel(robot_state, -robot_state.move_dist, 0)
                travel(robot_state, 0, robot_state.turn_angle)
            else:
                robot_state.motor_controller.motors(0, 0)
                # TODO: change this to pid or time based. NEED TO MAKE SURE ROBOT DOESN'T BREAK WHEN GOING FROM
                #  POS VEL TO NEG VEL IN A SHORT PERIOD OF TIME -> ramp down prob
        else:
            if robot_state.is_sim:
                travel(robot_state, robot_state.move_dist, 0)
            else:
                # TODO: determine what vel to run this at
                robot_state.motor_controller.motors(0, 0)
                time.sleep(10)
        dt += 10  # accumulation in time in ms
        exit_boolean = (dt > time_limit)
    robot_state.phase = Phase.COMPLETE
    phase_change  # TODO: CHANGE the next phase to return
    return None


# JULIE NOTE: I DONT THINK THIS FUNCTION WILL WORK. WE CAN'T TURN IN PLACE. 
# I am leaving this here because the obstacle avoidance code currently uses it
def turn_to_target_heading(robot_state, target_heading, allowed_heading_error, database):
        """
        Turns robot in-place to target heading + or - allowed_heading_error, utilizing heading PID.
        Arguments:
            target_heading: the heading in radians the robot should approach at the end of in-place rotation.
            allowed_heading_error: the maximum error in radians a robot can have to target heading while turning in
                place.
        """

        predicted_state = robot_state.state  # this will come from Kalman Filter

        abs_heading_error = abs(target_heading - float(predicted_state[2]))
        robot_state.head_pid.reset_integral()
        while abs_heading_error > allowed_heading_error:
            theta_error = target_heading - robot_state.state[2]
            w = robot_state.head_pid.update(theta_error)  # angular velocity
            _, limited_cmd_w = limit_cmds(
                0, w, robot_state.max_velocity, robot_state.radius)

            travel(robot_state, 0, robot_state.time_step * limited_cmd_w)
            if not robot_state.is_sim:
                robot_state.motor_controller.spin_motors(limited_cmd_w, 0)
                time.sleep(10)

            # Get state after movement:
            predicted_state = robot_state.state  # this will come from Kalman Filter

            database.update_data(
                "state", robot_state.state[0], robot_state.state[1], robot_state.state[2])

            abs_heading_error = abs(target_heading - float(predicted_state[2]))

