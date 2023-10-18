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
    robot_state.prev_phase = Phase.TRAVERSE
    control_mode = robot_state.control_mode
    if control_mode == ControlMode.LAWNMOWER:
        return traverse_standard(robot_state, mission_state.waypoints_to_visit,
                                 mission_state.allowed_dist_error, database)
    elif control_mode == ControlMode.ROOMBA:
        robot_state.enable_obstacle_avoidance = False
        robot_state = traverse_roomba(robot_state, mission_state.base_station.position,
                                      mission_state.time_limit, mission_state.roomba_radius, database)
        robot_state.enable_obstacle_avoidance = True
        return robot_state, None


def traverse_standard(robot_state, unvisited_waypoints, allowed_dist_error, database):
    """ Move the robot by following the traversal path given by [unvisited_waypoints].
        Args:
            unvisited_waypoints ([Node list]): GPS traversal path in terms of meters for the current grid.
            allowed_dist_error (Double): the maximum distance in meters that the robot can be from a node for the
                robot to have "visited" that node
        Returns:
            unvisited_waypoints ([Node list]): GPS traversal path in terms of meters for the current grid.
    """
    if unvisited_waypoints is None:
        robot_state.phase = Phase.RETURN
        phase_change(robot_state)
        return robot_state, None
    curr_waypoint = unvisited_waypoints[0].get_m_coords()
    # TODO: add return when tank is full, etc
    robot_state.goal_location = curr_waypoint
    move_to_target_node(robot_state, curr_waypoint,
                        allowed_dist_error, database)
    unvisited_waypoints.popleft()

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
    robot_state.goal_location = target
    robot_state.loc_pid_x.reset_integral()
    robot_state.loc_pid_y.reset_integral()
    while (distance_away > allowed_dist_error) and not (robot_state.phase == Phase.AVOID_OBSTACLE):
        # Error in terms of latitude and longitude, NOT meters
        x_coords_error = target[0] - robot_state.state[0]
        y_coords_error = target[1] - robot_state.state[1]

        desired_angle = math.atan2(
            target[1] - robot_state.state[1], target[0] - robot_state.state[0])

        x_vel = robot_state.loc_pid_x.update(x_coords_error)
        y_vel = robot_state.loc_pid_y.update(y_coords_error)

        cmd_v, cmd_w = feedback_lin(
            predicted_state, x_vel, y_vel, robot_state.epsilon)

        turn_to_target_heading(robot_state, desired_angle, 0.01, database)
        cmd_v = np.array([math.sqrt(x_vel**2 + y_vel**2)])
        cmd_w = np.array([0])

        # clamping of velocities:
        (limited_cmd_v, limited_cmd_w) = limit_cmds(
            cmd_v, cmd_w, robot_state.max_velocity, robot_state.radius)
        # self.linear_v = limited_cmd_v[0]
        # self.angular_v = limited_cmd_w[0]

        robot_state.linear_v = limited_cmd_v[0]
        robot_state.angular_v = limited_cmd_w[0]
        travel(robot_state, robot_state.time_step *
               limited_cmd_v[0], robot_state.time_step * limited_cmd_w[0])
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
    if not robot_state.is_sim:
        robot_state.motor_controller.spin_motors(0, 0)


def travel(robot_state, delta_d, delta_phi):
    """
    Moves the robot delta_d dist and delta_phi angle
    Arguments:
        delta_d (float): target meters to travel 
        delta_phi (float): target radians to turn
    """
    # if it is a simulation, we update the robot state directly
    if robot_state.is_sim:
        robot_state.state = integrate_odom(
            robot_state.state, delta_d, delta_phi)
        robot_state.truthpose = np.append(
            robot_state.truthpose, np.transpose(robot_state.state), 0)
    # otherwise, we update the robot state using EKF
    else:
        update_state(robot_state, delta_d/robot_state.time_step,
                     delta_phi/robot_state.time_step)


def update_state(robot_state, velocity, omega):
    """
    Updates the state of the robot given the linear velocity (velocity) and angular velocity (omega) of the robot
    Args:
        velocity (Float): linear velocity of robot
        omega (Float): angular velocity of robot
    Returns:
        new_state/measurement (Tuple): new x, y, and heading of robot
    """
    robot_state.gps_data = (robot_state.gps.get_gps()[
                            "long"], robot_state.gps.get_gps()["lat"])

    robot_state.imu_data = robot_state.imu.get_gps()

    x = get_vincenty_x(robot_state.init_gps, robot_state.gps_data)
    y = get_vincenty_y(robot_state.init_gps, robot_state.gps_data)

    heading = math.degrees(math.atan2(
        robot_state.imu_data["mag"]["y"], robot_state.imu_data["mag"]["x"]))

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


def traverse_roomba(robot_state, base_station_loc, time_limit, roomba_radius, database):
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
    dt = 10
    accumulated_time = 0
    # TODO: battery_limit, time_limit, tank_capacity is full
    exit_boolean = False
    if not robot_state.is_sim:
        from electrical.ultrasonic_sensor import Ultrasonic
        front_ultrasonic = Ultrasonic(0)
    curr_pos = past_pos = robot_state.state
    while not exit_boolean:
        # sensor should not detect something in the robot
        if not robot_state.is_sim and front_ultrasonic.distance() < robot_state.front_sensor_offset:
            robot_state.phase = Phase.FAULT
            phase_change(robot_state)
            return None
        curr_pos = robot_state.state
        vel = calculate_dist(past_pos[:2], curr_pos[:2])/(dt/1000)
        if robot_state.using_ekf:
            [new_x, new_y, _] = robot_state.ekf.get_predicted_state(
                curr_pos, [vel * dt/1000, 0])
        else:
            [new_x, new_y, _] = curr_pos + vel * dt/1000
        next_radius = calculate_dist(base_station_loc, (new_x, new_y))
        robot_state.goal_location = (new_x, new_y)
        # if moving will cause the robot to move through the obstacle
        if not robot_state.is_sim:
            # if obstacle detected; have to make sure move_dist < detect_obstacle_range; otherwise, robot might hit obstacle before we go back to this loop
            is_next_timestep_blocked = front_ultrasonic.distance(
            ) < robot_state.detect_obstacle_range
        else:
            next_radius_2 = calculate_dist(
                base_station_loc, curr_pos[:2])
            threshold = 1
            is_next_timestep_blocked = abs(
                next_radius_2 - roomba_radius) < threshold
        # sensor should not detect something in the robot
        if (next_radius > roomba_radius) or is_next_timestep_blocked:
            # this needs to be synchronous/PID'ed, otherwise, turn might be called while robot moving forward
            allowed_dist_error = 0.01
            allowed_heading_error = 0.01
            # double check that robot wont collide behind
            move_to_target_node(robot_state, (curr_pos[0] - robot_state.move_dist * math.cos(curr_pos[2]), curr_pos[1] - robot_state.move_dist * math.sin(curr_pos[2])),
                                allowed_dist_error, database)
            turn_to_target_heading(
                robot_state, curr_pos[2] + robot_state.turn_angle, allowed_heading_error, database)
        else:
            travel(robot_state, robot_state.move_dist, 0)
            database.update_data(
                "state", robot_state.state[0], robot_state.state[1], robot_state.state[2])
            if not robot_state.is_sim:
                # TODO: determine what vel to run this at
                robot_state.motor_controller.motors(0, 0)
        if not robot_state.is_sim:
            time.sleep(dt)
        accumulated_time += dt  # accumulation in time in ms
        exit_boolean = (accumulated_time > time_limit)
        past_pos = curr_pos
    robot_state.phase = Phase.RETURN
    phase_change(robot_state)
    if not robot_state.is_sim:
        robot_state.motor_controller.spin_motors(0, 0)
    return robot_state


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
    robot_state.enable_obstacle_avoidance = False  # we dont want the robot to avoid obstacle here
    predicted_state = robot_state.state  # this will come from Kalman Filter

    abs_heading_error = abs(target_heading - float(predicted_state[2]))
    robot_state.head_pid.reset_integral()
    while abs_heading_error > allowed_heading_error:
        theta_error = target_heading - robot_state.state[2]
        # TODO: change heading pid to give optimal velocity depending on whether the robot should turn left or right
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
    # re-enable after finishing turning
    robot_state.enable_obstacle_avoidance = True
    if not robot_state.is_sim:
        robot_state.motor_controller.spin_motors(0, 0)
