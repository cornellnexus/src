import unittest
from engine.robot_logic.robot import Robot
from engine.robot_state import Robot_State
import numpy as np


class TestIsOnLine(unittest.TestCase):

    def test_true(self):
        rb_state = Robot_State(xpos=0, ypos=0, heading=0,
                               epsilon=0.2, max_velocity=0.5, radius=0.2)
        r2d2 = Robot(rb_state)
        self.assertEqual(True, r2d2.is_on_line((0, 0), (5, 5), .001))
        rb_state.state = [7, 7, 0]
        self.assertEqual(True, r2d2.is_on_line((0, 0), (5, 5), .001))
        rb_state.state = [-10, 1, 1]
        self.assertEqual(True, r2d2.is_on_line((1, 1), (4, 1), .001))
        rb_state.state = [-1, 14, 2]
        self.assertEqual(True, r2d2.is_on_line((-1, -1), (-1, -4), .001))
        rb_state.state = [8, 11, 3]
        self.assertEqual(True, r2d2.is_on_line((-1, -1), (2, 3), .001))
        rb_state.state = [-1.5, -1.5, 4]
        self.assertEqual(True, r2d2.is_on_line((1.5, 2.5), (4.5, 6.5), .001))

        # test thetas
        rb_state.state = [7, 0, .5]
        self.assertEqual(True, r2d2.is_on_line(
            (0, 0), (5, 0), .001))  # theta = 0
        rb_state.state = [7, 7, .5]
        self.assertEqual(True, r2d2.is_on_line(
            (0, 0), (5, 5), .001))  # theta = pi/4
        rb_state.state = [0, 7, .5]
        self.assertEqual(True, r2d2.is_on_line(
            (0, 0), (0, 5), .001))  # theta = pi/2
        rb_state.state = [-7, 7, .5]
        self.assertEqual(True, r2d2.is_on_line(
            (0, 0), (-5, 5), .001))  # theta = 3 * pi/4
        rb_state.state = [-7, 0, .5]
        self.assertEqual(True, r2d2.is_on_line(
            (0, 0), (-5, 0), .001))  # theta = pi
        rb_state.state = [-7, -7, .5]
        self.assertEqual(True, r2d2.is_on_line(
            (0, 0), (-5, -5), .001))  # theta = 5 * pi/4
        rb_state.state = [0, -7, .5]
        self.assertEqual(True, r2d2.is_on_line(
            (0, 0), (0, -5), .001))  # theta = 3 * pi/2
        rb_state.state = [7, -7, .5]
        self.assertEqual(True, r2d2.is_on_line(
            (0, 0), (5, -5), .001))  # theta = 7 * pi/4

    def test_false(self):
        rb_state = Robot_State(xpos=0, ypos=0, heading=0,
                               epsilon=0.2, max_velocity=0.5, radius=0.2)
        r2d2 = Robot(rb_state)

        # test higher thresholds
        rb_state.state = [7, 7+5, -1]
        self.assertEqual(False, r2d2.is_on_line((0, 0), (5, 5), .001))
        rb_state.state = [-10, 1+5, -2]
        self.assertEqual(False, r2d2.is_on_line((1, 1), (4, 1), .001))
        rb_state.state = [-1 - 5, 14, -3]
        self.assertEqual(False, r2d2.is_on_line((-1, -1), (-1, -4), .001))
        rb_state.state = [8, 11+4, -4]
        self.assertEqual(False, r2d2.is_on_line((-1, -1), (2, 3), .001))
        rb_state.state = [-1.5, -1.5 - 3.5, -5]
        self.assertEqual(False, r2d2.is_on_line((1.5, 2.5), (4.5, 6.5), .001))

        # test thetas
        rb_state.state = [7, 2, .5]
        self.assertEqual(False, r2d2.is_on_line(
            (0, 0), (5, 0), .001))  # theta = 0
        rb_state.state = [7, 9, .5]
        self.assertEqual(False, r2d2.is_on_line(
            (0, 0), (5, 5), .001))  # theta = pi/4
        rb_state.state = [2, 7, .5]
        self.assertEqual(False, r2d2.is_on_line(
            (0, 0), (0, 5), .001))  # theta = pi/2
        rb_state.state = [-9, 7, .5]
        self.assertEqual(False, r2d2.is_on_line(
            (0, 0), (-5, 5), .001))  # theta = 3 * pi/4
        rb_state.state = [-7, -2, .5]
        self.assertEqual(False, r2d2.is_on_line(
            (0, 0), (-5, 0), .001))  # theta = pi
        rb_state.state = [-7, -5, .5]
        self.assertEqual(False, r2d2.is_on_line(
            (0, 0), (-5, -5), .001))  # theta = 5 * pi/4
        rb_state.state = [-2, -7, .5]
        self.assertEqual(False, r2d2.is_on_line(
            (0, 0), (0, -5), .001))  # theta = 3 * pi/2
        rb_state.state = [5, -7, .5]
        self.assertEqual(False, r2d2.is_on_line(
            (0, 0), (5, -5), .001))  # theta = 7 * pi/4


if __name__ == '__main__':
    unittest.main()
