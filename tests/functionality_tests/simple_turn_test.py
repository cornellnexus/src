# THIS IS A TESTING SCRIPT FOR TURN_TO_TARGET_HEADING. USE THIS SCRIPT WHEN PHYSICALLY TESTING THE ROBOT AND CHANGE THE test_state TO THE STATE YOU NEED

from engine.robot_state import Robot_State
from engine.robot import Robot
from engine.robot_logic.traversal import simpler_turn_to_target_heading, limit_cmds, travel
from engine.database import DataBase
import math
import time

# Simpler version of the turn function without using PID
def simpler_turn_to_target_heading(
    robot_state, target_heading, allowed_heading_error, database
):
    """
    Turns robot in-place to target heading + or - allowed_heading_error
        target_heading: the heading in radians the robot should approach at the end of in-place rotation.
        allowed_heading_error: the maximum error in radians a robot can have to target heading while turning in
            place.
    """
    # we dont want the robot to avoid obstacle here
    robot_state.enable_obstacle_avoidance = False

    target_heading = target_heading % (2 * math.pi) 
    target_heading = (target_heading + 2 * math.pi) % (2 * math.pi) # ensures target_heading is positive

    # Assumes robot_state.state[2] is from 0 to 2 pi
    if abs(target_heading - float(robot_state.state[2])) > math.pi:
        if target_heading > math.pi:
            target_heading -= 2 * math.pi # e.g. if target was 359 degrees and current was 1, now target would be -1
        else:
            target_heading += 2 * math.pi # e.g. if target was 1 degree and current was 359, now target would be 361


    while abs(target_heading - float(robot_state.state[2])) > allowed_heading_error:
        theta_error = target_heading - robot_state.state[2]
        w = theta_error  # angular velocity
        _, limited_cmd_w = limit_cmds(
            0, w, robot_state.max_velocity, robot_state.radius
        )

        travel(robot_state, 0, robot_state.time_step * limited_cmd_w)
        if not robot_state.is_sim:
            robot_state.motor_controller.spin_motors(limited_cmd_w, 0)
            time.sleep(10)

        database.update_data(
            "state", robot_state.state[0], robot_state.state[1], robot_state.state[2]
        )

    # re-enable after finishing turning
    robot_state.enable_obstacle_avoidance = True
    
if __name__ == "__main__":
    # 0 does nothing, 1 is a 90 degree turn, 2 is a 180 degree turn, 3 is a -90 degree turn, 4 is a negative 180 degree turn
    test_state = 0
    rb_state = Robot_State(x_pos=0, y_pos = 0, max_velocity = 0.5, radius = 0.2)
    r2d2 = Robot(rb_state)
    database = DataBase(r2d2)
    r2d2.execute_setup()
    rb_state.enable_obstacle_avoidance = False

    if test_state == 0:
        simpler_turn_to_target_heading(rb_state, 0, 2, database)

    elif test_state == 1:
        simpler_turn_to_target_heading(rb_state, math.pi / 2, 2, database)

    elif test_state == 2:
        simpler_turn_to_target_heading(rb_state, math.pi, 2, database)
    
    elif test_state == 3:
        simpler_turn_to_target_heading(rb_state, -math.pi / 2, 2, database)
    
    elif test_state == 4:
        simpler_turn_to_target_heading(rb_state, -math.pi, 2, database)
    