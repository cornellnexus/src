import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import animation as animation
from matplotlib import patches as patch
import unittest

from engine.grid import *
from engine.kinematics import limit_cmds, feedback_lin, integrate_odom, get_vincenty_x, get_vincenty_y
from engine.node import *
from engine.pid_controller import PID
from engine.robot_logic.robot import Robot
from engine.sim_trajectory import *
from engine.user_utils import *

"""
THIS FILE IS OUTDATED. OUR IMPLEMENTATION OF TRAVERSAL HAS CHANGED. 
Running this file will yield errors. 
"""

target = [0, 0]
add_to_x = False
gps_noise_range = .3


# lat_min, lat_max, long_min, long_max = get_coord_inputs()
# current_position = [lat_min, long_min]

class TestGenerateNodes(unittest.TestCase):
    def test_no_noise_all_nodes(self):
        r2d2 = Robot(-5, -10, math.pi / 2)
        NOISE_RANGE = 0.0

        goals = np.array([[-5, -10], [-5, -5], [-5, 0], [-5, 5], [-5, 10], [0, 10], [0, 5], \
                          [0, 0], [0, -5], [0, -10], [5, -10], [5, -5], [5, 0], [5, 5], [5, 10], [10, 10], [10, 5], \
                          [10, 0], [10, -5], [10, -10]])

        Kp = 1
        Ki = 0.1
        Kd = 0.1

        all_nodes_traveled_to = simulation(r2d2, NOISE_RANGE, goals, Kp, Ki, Kd)
        self.assertEqual([True] * len(goals.tolist()), all_nodes_traveled_to)

    def test_no_noise_first_target(self):
        r2d2 = Robot(-5, -10, math.pi / 2)
        NOISE_RANGE = 0.0

        goals = np.array([[-5, -10], [-5, -5], [-5, 0], [-5, 5], [-5, 10], [0, 10], [0, 5], \
                          [0, 0], [0, -5], [0, -10], [5, -10], [5, -5], [5, 0], [5, 5], [5, 10], [10, 10], [10, 5], \
                          [10, 0], [10, -5], [10, -10]])

        Kp = 1
        Ki = 0.1
        Kd = 0.1

        all_nodes_traveled_to = simulation(r2d2, NOISE_RANGE, goals, Kp, Ki, Kd)
        self.assertEqual([True] * 2, all_nodes_traveled_to[0:2])

    def test_no_noise_first_turn(self):
        r2d2 = Robot(-5, -10, math.pi / 2)
        NOISE_RANGE = 0.0

        goals = np.array([[-5, -10], [-5, -5], [-5, 0], [-5, 5], [-5, 10], [0, 10], [0, 5], \
                          [0, 0], [0, -5], [0, -10], [5, -10], [5, -5], [5, 0], [5, 5], [5, 10], [10, 10], [10, 5], \
                          [10, 0], [10, -5], [10, -10]])

        Kp = 1
        Ki = 0.1
        Kd = 0.1

        all_nodes_traveled_to = simulation(r2d2, NOISE_RANGE, goals, Kp, Ki, Kd)
        self.assertEqual([True] * 3, all_nodes_traveled_to[0:3])


class TestPlottingFunctions(unittest.TestCase):
    def test_waypoints_to_array(self):
        # Create node_list and corresponding numpy array
        node_list = []
        ans_node_list = []
        for i in range(0, 22):
            for j in range(0, 10):
                node_list.append(Node(i, j))
                ans_node_list.append([float(i), float(j)])
        ans_node_list = np.array(ans_node_list)

        # Check equality
        result = np.array_equal(ans_node_list, waypoints_to_array(node_list))
        self.assertEqual(True, result)

    def test_get_plot_boundaries(self):
        # CASE: Eng Quad
        grid_eng = Grid(42.444250, 42.444599, -76.483682, -76.483276).meters_grid
        self.assertEqual(([0.0, 32.085], [0, 37.961]), get_plot_boundaries(grid_eng, 0))
        self.assertEqual(([-5.0, 37.085], [-5.0, 42.961]), get_plot_boundaries(grid_eng, 5))

        # Calculates bounds based on input grid information
        def calc_ans(lat_min, long_min, grid, delta):
            # Need true_max_lat to account for steps in grid
            true_max_lat = grid.true_lat_max
            true_max_long = grid.true_long_max
            # Calculate distance in meters between smallest coordinate and largest coordinate
            coord1, coord2 = [lat_min, long_min], [true_max_lat, true_max_long]
            x_dist = get_vincenty_x(coord1, coord2)
            y_dist = get_vincenty_y(coord1, coord2)
            # factor in desired delta spacing
            return [0 - delta, x_dist + delta], [0 - delta, y_dist + delta]

        # CASE: Checking unit meter conversion (no extra spacing)
        lat_min = 0.0
        long_min = 0.0
        lat_max = 0.0001
        long_max = 0.0001
        grid_simple = Grid(lat_min, lat_max, long_min, long_max)
        self.assertEqual(calc_ans(lat_min, long_min, grid_simple, 0), \
                         get_plot_boundaries(grid_simple.meters_grid, 0))

        # CASE: Checking unit meter conversion (with extra spacing)
        lat_min = 1.0
        long_min = 1.0
        lat_max = 1.0003
        long_max = 1.0003
        grid_spacing = Grid(lat_min, lat_max, long_min, long_max)
        self.assertEqual(calc_ans(lat_min, long_min, grid_spacing, 4), \
                         get_plot_boundaries(grid_spacing.meters_grid, 4))

        # CASE: Checking unit meter conversion (diff lat longs)
        lat_min = 0.0001
        long_min = 0.0
        lat_max = 0.0003
        long_max = 0.0002
        grid_diff = Grid(lat_min, lat_max, long_min, long_max)
        self.assertEqual(calc_ans(lat_min, long_min, grid_diff, 3), \
                         get_plot_boundaries(grid_diff.meters_grid, 3))


if __name__ == '__main__':
    unittest.main()
