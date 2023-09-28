import time
import math

from engine.phase import Phase
from constants.definitions import *

from engine.robot_logic.robot_helpers import phase_change, calculate_dist
from engine.robot_logic.traversal import turn_to_target_heading
from engine.is_raspberrypi import is_raspberrypi
if is_raspberrypi():
    # Electrical library imports
    from electrical.ultrasonic_sensor import Ultrasonic


def avoid_obstacle_logic(robot_state):
    """ Execute bug 2 (straight line) obstacle avoidance algorithm
        Args:
            robot_state (init_threshold, goal_threshold, and threshold_to_recalculate_on_line are
            documented in robot_state)
        Returns:
            prev_phase (Phase)
    """
    # on_line_tolerance (float): threshold in meters for checking theta matches for the lines
    on_line_tolerance = robot_state.dist_to_goal

    init_x = robot_state.state[0]
    init_y = robot_state.state[1]
    init_pos = (init_x, init_y)

    # last position of the robot when it's still on the (line from its initial position to its goal positions)
    last_pos_on_line = init_pos
    # the robot's initial location is x,y and the init_threshold is d.
    # If the robot's new location x',y' is more than d distance from init_threshold,
    # we set has_left_init_threshold to true. Used to determine if the robot has left its initial location
    has_left_init_thresh = False
    has_traversed_boundary = False
    init_dist_to_goal = calculate_dist(robot_state.goal_location, curr_pos)
    while True:
        # fault has priority over obstacle avoidance
        if robot_state.phase == Phase.FAULT:
            robot_state.motor_controller.spin_motors(0, 0)
            return None

        curr_pos = (robot_state.state[0], robot_state.state[1])
        curr_dist_to_goal = calculate_dist(robot_state.goal_location, curr_pos)
        dist_from_init = calculate_dist(curr_pos, init_pos)
        is_on_line = is_on_line(
            robot_state, robot_state.goal_location, curr_pos, on_line_tolerance)

        if dist_from_init > robot_state.init_threshold:
            has_left_init_thresh = True
        # exits obstacle avoidance if robot close to goal
        if curr_dist_to_goal < robot_state.goal_threshold:
            robot_state.phase = robot_state.prev_phase
            phase_change(robot_state)
            robot_state.motor_controller.spin_motors(0, 0)
            return None
        elif has_traversed_boundary:
            robot_state.phase = Phase.FAULT  # cannot reach goal
            phase_change(robot_state)
            robot_state.motor_controller.spin_motors(0, 0)
            return None
        # bug 2
        elif is_on_line and (curr_dist_to_goal < init_dist_to_goal):
            if calculate_dist(curr_pos, last_pos_on_line) > robot_state.threshold_to_recalculate_on_line:
                robot_state.phase = robot_state.prev_phase
                phase_change(robot_state)
                robot_state.motor_controller.spin_motors(0, 0)
                return None
        else:
            execute_boundary_following(robot_state)
        if has_left_init_thresh:
            # we dont want the program to think that the boundary is untraversable
            #   when the robot is still in the threshold from where it started obstacle avoidance
            has_traversed_boundary = dist_from_init < robot_state.init_threshold
        time.sleep(10)  # don't hog the cpu


def is_on_line(robot_state, p1, p2, tolerance):
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
        (p2[1] - robot_state.state[1]), (p2[0] - robot_state.state[0]))
    difference = new_theta - line_theta
    # want to detect same line even if rotated 180
    if abs(difference) == math.pi:
        difference = 0
    return abs(difference) <= tolerance


def execute_boundary_following(robot_state):
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
    margin_to_front_obstacle = 5 + max(robot_state.width, robot_state.length)
    # When boundary following gap in wall but robot cannot fit (check both side and turn 180 and do boundary following again)
    if lf_ultrasonic.distance() < side_margin_sensor:
        curr_heading = robot_state.state[2]
        desired_heading = curr_heading - \
            math.pi if curr_heading > math.pi else curr_heading + math.pi
        # error is arbitrary
        heading_err = 10
        turn_to_target_heading(robot_state, desired_heading, heading_err)
    # if there is an obstacle in front of the robot, turn until there isn't
    elif front_ultrasonic.distance() < margin_to_front_obstacle:
        robot_state.motor_controller.spin_motors(0, robot_state.turn_angle)
    # if there is no obstacle in front of the robot but it's still in avoid obstacle because it's following the boundary
    else:
        rf_dist = rf_ultrasonic.distance()
        rb_dist = rb_ultrasonic.distance()
        # if the robot is approximately parallel to the robot, go forward
        if abs(rf_dist - rb_dist) < side_margin_sensor:
            robot_state.motor_controller.spin_motors(robot_state.move_dist, 0)
        # otherwise, turn until the robot is parallel
        else:
            direction = (rf_dist - rb_dist)/abs(rf_dist - rb_dist)
            robot_state.motor_controller.spin_motors(
                0, direction * robot_state.turn_angle)
    if not robot_state.is_sim:
        time.sleep(10)
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
