import numpy as np
import math
import unittest

from engine.grid import *
from engine.robot import Robot
from collections import deque
"""
THIS FILE IS OUTDATED. OUR IMPLEMENTATION OF TRAVERSAL HAS CHANGED. 
Running this file will yield errors. 
"""

target = [0, 0]
add_to_x = False
gps_noise_range = .3

class TestTraversalFunctions(unittest.TestCase):
    def test_waypoints_to_array(self):
        r2d2 = Robot(0, 0, math.pi / 2, epsilon=0.2, max_v=0.5, radius=0.2, init_phase=2)
        allowed_dist_error = 0.5
        grid=Grid(42.444250, 42.444599, -76.483682, -76.483276)
        grid_mode="borders"
        all_waypoints = grid.get_waypoints(grid_mode)
        waypoints_to_visit = deque(all_waypoints)
        unvisited_waypoints = r2d2.execute_traversal(waypoints_to_visit, allowed_dist_error)
        self.assertEquals(deque([]), unvisited_waypoints)


if __name__ == '__main__':
    unittest.main()