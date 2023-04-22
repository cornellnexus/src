import time
import math

from engine.phase import Phase
from constants.definitions import *

from engine.robot_logic.robot_helpers import set_phase, calculate_dist
from engine.robot_logic.traversal import turn_to_target_heading
from engine.is_raspberrypi import is_raspberrypi
if is_raspberrypi():
    # Electrical library imports
    from electrical.ultrasonic_sensor import Ultrasonic

def execute_avoid_obstacle(robot, on_line_tolerance, init_threshold=3., goal_threshold=3., threshold_to_recalculate_on_line=3.):
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

                threshold_to_recalculate_on_line (float): threshold in meters that we dont want to recalculate whether the robot 
                    is on the line to the goal.
                    This parameter needs to be tuned
                    Reasoning for this: without this condition, the robot could move closer to the goal (on a slant, not directly 
                    towards the goal) but still be on the line, making the robot keep exiting obstacle avoidance when its effectively
                    in the same position as before

            Returns:
                prev_phase (Phase)
        """
        init_x = robot.robot_state.state[0]
        init_y = robot.robot_state.state[1]
        init_pos = (init_x, init_y)

        # last position of the robot when it's still on the (line from its initial position to its goal positions)
        last_pos_on_line = init_pos
        # the robot's initial location is x,y and the init_threshold is d.
        # If the robot's new location x',y' is more than d distance from init_threshold,
        # we set has_left_init_threshold to true. Used to determine if the robot has left its initial location
        has_left_init_thresh = False
        has_traversed_boundary = False
        init_dist_to_goal = calculate_dist(robot.robot_state.goal_location, curr_pos)
        while True:
            # fault has priority over obstacle avoidance
            if robot.robot_state.phase == Phase.FAULT:
                return None
            
            curr_pos = (robot.robot_state.state[0], robot.robot_state.state[1])
            curr_dist_to_goal = calculate_dist(robot.robot_state.goal_location, curr_pos)
            dist_from_init = calculate_dist(curr_pos, init_pos)
            is_on_line = is_on_line(robot, robot.robot_state.goal_location, curr_pos, on_line_tolerance)
            
            if dist_from_init > init_threshold:
                has_left_init_thresh = True
            # exits obstacle avoidance if robot close to goal
            if curr_dist_to_goal < goal_threshold:
                robot = set_phase(robot, robot.robot_state.prev_phase)
                return None
            elif has_traversed_boundary:
                robot = set_phase(robot, Phase.FAULT)  # cannot reach goal
                return None
            # bug 2
            elif is_on_line and (curr_dist_to_goal < init_dist_to_goal):
                if calculate_dist(curr_pos, last_pos_on_line) > threshold_to_recalculate_on_line:
                    robot = set_phase(robot, robot.robot_state.prev_phase)
                    return None
            else:
                execute_boundary_following(robot)
            if has_left_init_thresh:
                # we dont want the program to think that the boundary is untraversable
                #   when the robot is still in the threshold from where it started obstacle avoidance
                has_traversed_boundary = dist_from_init < init_threshold
            time.sleep(10)  # don't hog the cpu

def is_on_line(robot, p1, p2, tolerance):
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
            (p2[1] - robot.robot_state.state[1]), (p2[0] - robot.robot_state.state[0]))
        difference = new_theta - line_theta
        # want to detect same line even if rotated 180
        if abs(difference) == math.pi:
            difference = 0
        return abs(difference) <= tolerance

def execute_boundary_following(robot):
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
    margin_to_front_obstacle = 5 + max(robot.robot_state.width, robot.robot_state.length)
    # When boundary following gap in wall but robot cannot fit (check both side and turn 180 and do boundary following again)
    if lf_ultrasonic.distance() < side_margin_sensor:
        curr_heading = robot.robot_state.state[2]
        desired_heading = curr_heading - math.pi if curr_heading > math.pi else curr_heading + math.pi
        # error is arbitrary
        heading_err = 10
        turn_to_target_heading(robot, desired_heading, heading_err)
    # if there is an obstacle in front of the robot, turn until there isn't
    elif front_ultrasonic.distance() < margin_to_front_obstacle:
        robot.robot_state.motor_controller.spin_motors(0, robot.robot_state.turn_angle)
    # if there is no obstacle in front of the robot but it's still in avoid obstacle because it's following the boundary
    else:
        rf_dist = rf_ultrasonic.distance()
        rb_dist = rb_ultrasonic.distance()
        # if the robot is approximately parallel to the robot, go forward
        if abs(rf_dist - rb_dist) < side_margin_sensor:
            robot.robot_state.motor_controller.spin_motors(robot.robot_state.move_dist, 0)
        # otherwise, turn until the robot is parallel
        else:
            direction = (rf_dist - rb_dist)/abs(rf_dist - rb_dist)
            robot.robot_state.motor_controller.spin_motors(0, direction * robot.robot_state.turn_angle)
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
    
