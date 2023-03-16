import unittest
from engine.robot import Robot
import random
import numpy as np
import math


class TestIsOnLine(unittest.TestCase):

    def test_true(self):
        r2d2 = Robot(0, 0, 0, epsilon=0.2, max_v=0.5, radius=0.2, init_phase=2, width=785,
                     front_ultrasonic=None, lb_ultrasonic=None, lf_ultrasonic=None, rb_ultrasonic=None,
                     rf_ultrasonic=None)
        self.assertEqual(True, r2d2.is_on_line((0, 0), (5, 5), .001))
        r2d2.state = [7, 7, 0]
        self.assertEqual(True, r2d2.is_on_line((0, 0), (5, 5), .001))
        r2d2.state = [-10, 1, 1]
        self.assertEqual(True, r2d2.is_on_line((1, 1), (4, 1), .001))
        r2d2.state = [-1, 14, 2]
        self.assertEqual(True, r2d2.is_on_line((-1, -1), (-1, -4), .001))
        r2d2.state = [8, 11, 3]
        self.assertEqual(True, r2d2.is_on_line((-1, -1), (2, 3), .001))
        r2d2.state = [-1.5, -1.5, 4]
        self.assertEqual(True, r2d2.is_on_line((1.5, 2.5), (4.5, 6.5), .001))

        # test thetas
        r2d2.state = [7, 0, .5]
        self.assertEqual(True, r2d2.is_on_line(
            (0, 0), (5, 0), .001))  # theta = 0
        r2d2.state = [7, 7, .5]
        self.assertEqual(True, r2d2.is_on_line(
            (0, 0), (5, 5), .001))  # theta = pi/4
        r2d2.state = [0, 7, .5]
        self.assertEqual(True, r2d2.is_on_line(
            (0, 0), (0, 5), .001))  # theta = pi/2
        r2d2.state = [-7, 7, .5]
        self.assertEqual(True, r2d2.is_on_line(
            (0, 0), (-5, 5), .001))  # theta = 3 * pi/4
        r2d2.state = [-7, 0, .5]
        self.assertEqual(True, r2d2.is_on_line(
            (0, 0), (-5, 0), .001))  # theta = pi
        r2d2.state = [-7, -7, .5]
        self.assertEqual(True, r2d2.is_on_line(
            (0, 0), (-5, -5), .001))  # theta = 5 * pi/4
        r2d2.state = [0, -7, .5]
        self.assertEqual(True, r2d2.is_on_line(
            (0, 0), (0, -5), .001))  # theta = 3 * pi/2
        r2d2.state = [7, -7, .5]
        self.assertEqual(True, r2d2.is_on_line(
            (0, 0), (5, -5), .001))  # theta = 7 * pi/4

    def test_false(self):
        r2d2 = Robot(0, 0, 0, epsilon=0.2, max_v=0.5, radius=0.2, init_phase=2, width=785,
                     front_ultrasonic=None, lb_ultrasonic=None, lf_ultrasonic=None, rb_ultrasonic=None,
                     rf_ultrasonic=None)

        # test higher thresholds
        r2d2.state = [7, 7+5, -1]
        self.assertEqual(False, r2d2.is_on_line((0, 0), (5, 5), .001))
        r2d2.state = [-10, 1+5, -2]
        self.assertEqual(False, r2d2.is_on_line((1, 1), (4, 1), .001))
        r2d2.state = [-1 - 5, 14, -3]
        self.assertEqual(False, r2d2.is_on_line((-1, -1), (-1, -4), .001))
        r2d2.state = [8, 11+4, -4]
        self.assertEqual(False, r2d2.is_on_line((-1, -1), (2, 3), .001))
        r2d2.state = [-1.5, -1.5 - 3.5, -5]
        self.assertEqual(False, r2d2.is_on_line((1.5, 2.5), (4.5, 6.5), .001))

        # test thetas
        r2d2.state = [7, 2, .5]
        self.assertEqual(False, r2d2.is_on_line(
            (0, 0), (5, 0), .001))  # theta = 0
        r2d2.state = [7, 9, .5]
        self.assertEqual(False, r2d2.is_on_line(
            (0, 0), (5, 5), .001))  # theta = pi/4
        r2d2.state = [2, 7, .5]
        self.assertEqual(False, r2d2.is_on_line(
            (0, 0), (0, 5), .001))  # theta = pi/2
        r2d2.state = [-9, 7, .5]
        self.assertEqual(False, r2d2.is_on_line(
            (0, 0), (-5, 5), .001))  # theta = 3 * pi/4
        r2d2.state = [-7, -2, .5]
        self.assertEqual(False, r2d2.is_on_line(
            (0, 0), (-5, 0), .001))  # theta = pi
        r2d2.state = [-7, -5, .5]
        self.assertEqual(False, r2d2.is_on_line(
            (0, 0), (-5, -5), .001))  # theta = 5 * pi/4
        r2d2.state = [-2, -7, .5]
        self.assertEqual(False, r2d2.is_on_line(
            (0, 0), (0, -5), .001))  # theta = 3 * pi/2
        r2d2.state = [5, -7, .5]
        self.assertEqual(False, r2d2.is_on_line(
            (0, 0), (5, -5), .001))  # theta = 7 * pi/4


if __name__ == '__main__':
    unittest.main()
