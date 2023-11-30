import numpy as np
import math
import unittest
import time

from engine.database import DataBase
from engine.grid import *
from engine.robot import Robot
from engine.robot_logic.traversal import limit_cmds, travel, turn_to_target_heading
from engine.robot_state import Robot_State

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

    target_heading = (target_heading + math.pi) % (2 * math.pi) - math.pi

    while abs(target_heading - float(robot_state.state[2])) >= allowed_heading_error:
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


target = [0, 0]
add_to_x = False
gps_noise_range = 0.3


class TestMoveToTargetNode(unittest.TestCase):
    def test1(self):
        r2d2_state = Robot_State(
            xpos=32.444250,
            ypos=-44.483682,
            heading=0,
            max_velocity=0.5,
            radius=0.2
        )
        r2d2 = Robot(robot_state=r2d2_state)
        database = DataBase(r2d2)

        allowed_heading_error = 0.1
        simpler_turn_to_target_heading(
            r2d2_state, math.pi, allowed_heading_error, database
        )

    def test2(self):
        r2d2_state = Robot_State(
            xpos=42.444250,
            ypos=-54.483682,
            heading=math.pi / 2,
            max_velocity=0.5,
            radius=0.2
        )
        r2d2 = Robot(robot_state=r2d2_state)
        database = DataBase(r2d2)

        allowed_heading_error = 0.1
        simpler_turn_to_target_heading(
            r2d2_state, math.pi / 2, allowed_heading_error, database
        )

    def test3(self):
        r2d2_state = Robot_State(
            xpos=12.444250,
            ypos=-64.483682,
            heading=0,
            max_velocity=0.5,
            radius=0.2,
        )
        r2d2 = Robot(robot_state=r2d2_state)
        database = DataBase(r2d2)

        allowed_heading_error = 0.1
        simpler_turn_to_target_heading(
            r2d2_state, -math.pi / 2, allowed_heading_error, database
        )
    
    def test4(self):
        r2d2_state = Robot_State(
            xpos=-32.444250,
            ypos=44.483682,
            heading=math.pi / 2,
            max_velocity=0.5,
            radius=0.2,
        )
        r2d2 = Robot(robot_state=r2d2_state)
        database = DataBase(r2d2)
        allowed_heading_error = 0.1
        simpler_turn_to_target_heading(
            r2d2_state, -math.pi, allowed_heading_error, database
        )
    
    def test5(self):
        r2d2_state = Robot_State(
            xpos=22.444250,
            ypos=44.483682,
            heading=0,
            max_velocity=0.5,
            radius=0.2,
        )
        r2d2 = Robot(robot_state=r2d2_state)
        database = DataBase(r2d2)
        allowed_dist_error = 0.1
        simpler_turn_to_target_heading(
            r2d2_state, math.pi / 2, allowed_dist_error, database
        )

# Takes approx. 9 seconds for 5 tests

if __name__ == "__main__":
    unittest.main()
