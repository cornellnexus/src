import numpy as np
import math
import unittest

from engine.database import DataBase
from engine.grid import *
from engine.robot import Robot
from collections import deque

target = [0, 0]
add_to_x = False
gps_noise_range = .3

class TestMoveToNodeFunction(unittest.TestCase):

    def test_successfully_traveled(self):
        """
        This is a test to see if the robot's final position after calling move_to_target_node 
        is within the allowed distance error.

        This robot is initialized to the correct heading and only needs to move in a straight line. 
        In this specific test, our target node is .0005 latitude away, with position_kp and heading_kp 
        constants for our P controller being 0.1. Because the robot is not turning, the heading PID does
        not seem to have a great effect in this test case.
        
        """
        r2d2 = Robot(42.444250, -76.483682, 0, epsilon=0.2, max_v=0.5, radius=0.2, init_phase=2, position_kp = .1, heading_kp = 0.1, position_kd=0.00, heading_kd=0.0)
        database = DataBase(r2d2)

        #Latitude is changing, longitude should stay consistent
        # 42.444400 - 42.444250 = 0.000150
        # target = (52.445400, -76.483682)
        target = (42.444750, -76.483682)
        allowed_dist_error = 0.0001
        print("here")
        r2d2.move_to_target_node(target, allowed_dist_error, database)

    def test_failed_to_travel(self):
        pass


# Takes approx. 9 seconds for 5 tests
# class TestTraverseStandardFunctions(unittest.TestCase):

#     def test_init(self):
#         r2d2 = Robot(0, 0, math.pi / 2, epsilon=0.2, max_v=0.5, radius=0.2, init_phase=2)
#         database = DataBase(r2d2)

#         allowed_dist_error = 0.5
#         grid = Grid(42.444250, 42.444599, -76.483682, -76.483276)
#         grid_mode = "lawn_border"
#         all_waypoints = grid.get_waypoints(grid_mode)
#         waypoints_to_visit = deque(all_waypoints)
#         unvisited_waypoints = r2d2.traverse_standard(waypoints_to_visit, allowed_dist_error, database)
#         self.assertEqual(deque([]), unvisited_waypoints)

#     def test_different_init_robot_pos(self):
#         r2d2 = Robot(42, -76, math.pi / 2, epsilon=0.2, max_v=0.5, radius=0.2, init_phase=2)
#         database = DataBase(r2d2)

#         allowed_dist_error = 0.5
#         grid = Grid(42.444250, 42.444599, -76.483682, -76.483276)
#         grid_mode = "lawn_border"
#         all_waypoints = grid.get_waypoints(grid_mode)
#         waypoints_to_visit = deque(all_waypoints)
#         unvisited_waypoints = r2d2.traverse_standard(waypoints_to_visit, allowed_dist_error, database)
#         self.assertEqual(deque([]), unvisited_waypoints)

#     def test_different_allowed_dist(self):
#         r2d2 = Robot(42, -76, math.pi / 2, epsilon=0.2, max_v=0.5, radius=0.2, init_phase=2)
#         database = DataBase(r2d2)

#         allowed_dist_error = 0.65
#         grid = Grid(42.444250, 42.444599, -76.483682, -76.483276)
#         grid_mode = "lawn_border"
#         all_waypoints = grid.get_waypoints(grid_mode)
#         waypoints_to_visit = deque(all_waypoints)
#         unvisited_waypoints = r2d2.traverse_standard(waypoints_to_visit, allowed_dist_error, database)
#         self.assertEqual(deque([]), unvisited_waypoints)

#     def test_different_heading(self):
#         r2d2 = Robot(42, -76, math.pi, epsilon=0.2, max_v=0.5, radius=0.2, init_phase=2)
#         database = DataBase(r2d2)

#         allowed_dist_error = 0.5
#         grid = Grid(42.444250, 42.444599, -76.483682, -76.483276)
#         grid_mode = "lawn_border"
#         all_waypoints = grid.get_waypoints(grid_mode)
#         waypoints_to_visit = deque(all_waypoints)
#         unvisited_waypoints = r2d2.traverse_standard(waypoints_to_visit, allowed_dist_error, database)
#         self.assertEqual(deque([]), unvisited_waypoints)

#     def test_with_minimal_noise(self):
#         r2d2 = Robot(42, -76, math.pi, epsilon=0.2, max_v=0.5, radius=0.2, init_phase=2, position_noise=0.05)
#         database = DataBase(r2d2)

#         allowed_dist_error = 0.5
#         grid = Grid(42.444250, 42.444599, -76.483682, -76.483276)
#         grid_mode = "lawn_border"
#         all_waypoints = grid.get_waypoints(grid_mode)
#         waypoints_to_visit = deque(all_waypoints)
#         unvisited_waypoints = r2d2.traverse_standard(waypoints_to_visit, allowed_dist_error, database)
#         self.assertEqual(deque([]), unvisited_waypoints)


if __name__ == '__main__':
    unittest.main()
