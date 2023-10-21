import copy
import math
import unittest
import numpy as np

from engine.robot import Robot
from engine.robot_logic.traversal import travel
from engine.robot_state import Robot_State

"""
Unit tests for robot.py
NOTE: MANY OF THESE FUNCTIONS ARE BASED SOLELY ON KINEMATIC EQUATIONS
        for more comprehensive testing, we should make sure these kinematic
        equations are correct!
"""


class TestNodes(unittest.TestCase):
    # set up parameters for robot
    x_pos = 5
    y_pos = 6
    heading = math.pi
    init_mode = "collect"
    is_sim = True
    init_charge = 100
    init_capacity = 100

    # deep copies because each test case changes the robot object,
    # and for some reason the tests are executed from bottom to top
    robot_state = Robot_State(
        xpos=x_pos,
        ypos=y_pos,
        heading=heading,
        epsilon=0.2,
        max_velocity=0.5,
        radius=0.2,
    )
    robot_one = Robot(robot_state=robot_state)
    robot_two = copy.deepcopy(robot_one)
    robot_three = copy.deepcopy(robot_one)
    robot_four = copy.deepcopy(robot_one)
    robot_five = copy.deepcopy(robot_one)
    robot_six = copy.deepcopy(robot_one)

    distance = 10
    turn_angle = math.pi / 2

    def test_travel(self):
        travel(self.robot_one.robot_state, self.distance, self.turn_angle)
        # Values calculated by hand based on kinematic equations
        new_x = round(-1.3662, 3)
        new_y = round(-0.366198, 3)
        new_theta = round(-math.pi / 2, 3)
        self.assertEqual(
            [[float(new_x)], [float(new_y)], [float(new_theta)]],
            np.round(self.robot_one.robot_state.state, 3).tolist(),
        )
        # was move_forward_default
        travel(self.robot_two.robot_state, self.distance, 0)
        new_x = self.robot_two.robot_state.state[0]
        new_y = self.robot_two.robot_state.state[1]
        new_theta = float(self.robot_two.robot_state.state[2])
        self.assertEqual(
            [[float(new_x)], [float(new_y)], [float(new_theta)]],
            self.robot_two.robot_state.state.tolist(),
        )
        # was test_turn
        travel(self.robot_four.robot_state, 0, math.pi / 2)
        self.assertEqual([-math.pi / 2], self.robot_four.robot_state.state[2])
        # was test_circle_turn
        original_angle = self.robot_five.robot_state.state[2]
        travel(self.robot_five.robot_state, 0, math.pi * 2)
        rounded_robot_five_state = [
            round(state[0], 3) for state in self.robot_five.robot_state.state
        ]
        self.assertEqual(
            [round(original_angle[0], 2)],
            round(rounded_robot_five_state[2] % math.pi, 2),
        )  # added round because travel clamps heading
        # was test_turn_with_time
        travel(self.robot_six.robot_state, 0, math.pi / 2)
        self.assertEqual([-math.pi / 2], self.robot_six.robot_state.state[2])

    # TODO: double check the calculations + test doesn't use time
    # def test_move_forward_with_time(self):
    #     # Values calculated by hand based on kinematic equations
    #     self.robot_three.move_forward(5)
    #     new_x = -45
    #     new_y = 5
    #     new_theta = float(self.robot_two.robot_state.state[2])
    #     self.assertEqual([[float(new_x)], [float(new_y)], [float(new_theta)]], self.robot_three.robot_state.state.tolist())


if __name__ == "__main__":
    unittest.main()
