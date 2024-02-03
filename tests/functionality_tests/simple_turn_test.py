import math
import unittest
import time

from engine.database import DataBase
from engine.robot import Robot
from engine.robot_logic.traversal import limit_cmds, travel
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

    while abs(target_heading - float(robot_state.state[2][0])) > allowed_heading_error:
        theta_error = target_heading - robot_state.state[2][0]
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

class TestSimplerTurnMethod(unittest.TestCase):
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

        allowed_heading_error = 0.01
        target = math.pi

        simpler_turn_to_target_heading(
            r2d2_state, target, allowed_heading_error, database
        )

        difference = (target - r2d2.robot_state.state[2][0]) % (2 * math.pi)
        self.assertAlmostEqual(difference, 0, places=2)

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

        allowed_heading_error = 0.01
        target = math.pi / 2

        simpler_turn_to_target_heading(
            r2d2_state, target, allowed_heading_error, database
        )

        difference = (target - r2d2.robot_state.state[2][0]) % (2 * math.pi)
        self.assertAlmostEqual(difference, 0, places=2)

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

        allowed_heading_error = 0.01
        target = -math.pi / 2

        simpler_turn_to_target_heading(
            r2d2_state, target, allowed_heading_error, database
        )

        difference = (target - r2d2.robot_state.state[2][0]) % (2 * math.pi)
        self.assertAlmostEqual(difference, 0, places=2)
    
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

        allowed_heading_error = 0.01
        target = -math.pi

        simpler_turn_to_target_heading(
            r2d2_state, target, allowed_heading_error, database
        )

        difference = (target - r2d2.robot_state.state[2][0]) % (2 * math.pi)
        self.assertAlmostEqual(difference, 0, places=2)
    
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

        allowed_dist_error = 0.01
        target = math.pi / 2

        simpler_turn_to_target_heading(
            r2d2_state, target, allowed_dist_error, database
        )

        difference = (target - r2d2.robot_state.state[2][0]) % (2 * math.pi)
        self.assertAlmostEqual(difference, 0, places=2)

# Takes approx. 0.001 seconds for 5 tests

def physicaltesting():
    # Change this from 0 to 4 for different (documented) behavior
    test_state = 0
    rb_state = Robot_State(0, 0, 0, 0.2, 0.5, 0.2)
    r2d2 = Robot(rb_state)
    database = DataBase(r2d2)
    r2d2.execute_setup()
    rb_state.enable_obstacle_avoidance = False

    # Robot should turn by 180 degrees
    if test_state == 0:
        simpler_turn_to_target_heading(math.pi, 0.01, database)

    # Robot should turn to the left by 90 degrees
    if test_state == 1:
        simpler_turn_to_target_heading(-math.pi / 2, 0.01, database)
    
    # Robot should turn to the right by 90 degrees
    if test_state == 2:
        simpler_turn_to_target_heading(math.pi / 2, 0.01, database)

    # Robot should do nothing
    if test_state == 3:
        simpler_turn_to_target_heading(0, 2, database)
    
    # Robot should turn to the right by 120 degrees
    if test_state == 4:
        simpler_turn_to_target_heading(2 * math.pi / 3, 0.01, database)

if __name__ == "__main__":
    # Change this to False for physical testing
    unittests = True

    if (unittests):
        unittest.main()
    else:
        physicaltesting()