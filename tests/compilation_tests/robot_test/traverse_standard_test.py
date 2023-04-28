import numpy as np
import math
import unittest

from engine.mission import ControlMode
from engine.database import DataBase
from engine.grid import *
from engine.robot import Robot
from engine.robot_state import Robot_State
from collections import deque

target = [0, 0]
add_to_x = False
gps_noise_range = .3


# Takes approx. 9 seconds for 5 tests
class TestTraverseStandardFunctions(unittest.TestCase):
    def test1(self):
        r2d2_state = Robot_State(xpos=42.444250, ypos=-76.483682, heading=0,
                                 epsilon=0.2, max_velocity=0.5, radius=0.2, phase=2, position_kp=.1, heading_kp=.1, position_kd=0.0, heading_kd=0.0
                                 )
        r2d2 = Robot(robot_state=r2d2_state)
        database = DataBase(r2d2)

        allowed_dist_error = 0.5
        r2d2.move_to_target_node(
            (42.444250, -66.483682), allowed_dist_error, database)

    def test2(self):
        r2d2_state = Robot_State(xpos=42.444250, ypos=-76.483682, heading=math.pi/2,
                                 epsilon=0.2, max_velocity=0.5, radius=0.2, phase=2, position_kp=.1, heading_kp=.1, position_kd=0.0, heading_kd=0.0
                                 )
        r2d2 = Robot(robot_state=r2d2_state)
        database = DataBase(r2d2)

        allowed_dist_error = 0.5
        r2d2.move_to_target_node(
            (52.444750, -76.483682), allowed_dist_error, database)

    def test3(self):
        r2d2_state = Robot_State(xpos=42.444250, ypos=-76.483682, heading=0,
                                 epsilon=0.2, max_velocity=0.5, radius=0.2, phase=2, position_kp=.1, heading_kp=.1, position_kd=0.0, heading_kd=0.0
                                 )
        r2d2 = Robot(robot_state=r2d2_state)
        database = DataBase(r2d2)
        allowed_dist_error = 0.5
        r2d2.move_to_target_node(
            (43.444250, -76.483682), allowed_dist_error, database)

    # def test_init(self):
    #     r2d2_state = Robot_State(xpos=0, ypos=0, heading=math.pi / 2,
    #                              epsilon=0.2, max_velocity=0.5, radius=0.2, phase=2, avoid_obstacle=False)
    #     r2d2 = Robot(robot_state=r2d2_state)
    #     database = DataBase(r2d2)

    #     allowed_dist_error = 0.5
    #     grid = Grid(42.444250, 42.444599, -76.483682, -76.483276)
    #     grid_mode = ControlMode.LAWNMOWER_B
    #     all_waypoints = grid.get_waypoints(grid_mode)
    #     waypoints_to_visit = deque(all_waypoints)
    #     print(waypoints_to_visit)
    #     unvisited_waypoints = r2d2.traverse_standard(
    #         waypoints_to_visit, allowed_dist_error, database)
    #     print(unvisited_waypoints)
    #     self.assertEqual(deque([]), unvisited_waypoints)

    # def test_different_init_robot_pos(self):
    #     r2d2_state = Robot_State(xpos=42, ypos=-76, heading=math.pi / 2,
    #                              epsilon=0.2, max_velocity=0.5, radius=0.2, phase=2, avoid_obstacle=False)
    #     r2d2 = Robot(robot_state=r2d2_state)
    #     database = DataBase(r2d2)

    #     allowed_dist_error = 0.5
    #     grid = Grid(42.444250, 42.444599, -76.483682, -76.483276)
    #     grid_mode = ControlMode.LAWNMOWER
    #     all_waypoints = grid.get_waypoints(grid_mode)
    #     waypoints_to_visit = deque(all_waypoints)
    #     unvisited_waypoints = r2d2.traverse_standard(
    #         waypoints_to_visit, allowed_dist_error, database)
    #     self.assertEqual(deque([]), unvisited_waypoints)

    # def test_different_allowed_dist(self):
    #     r2d2_state = Robot_State(xpos=42, ypos=-76, heading=math.pi / 2,
    #                              epsilon=0.2, max_velocity=0.5, radius=0.2, phase=2, avoid_obstacle=False)
    #     r2d2 = Robot(robot_state=r2d2_state)
    #     database = DataBase(r2d2)

    #     allowed_dist_error = 0.65
    #     grid = Grid(42.444250, 42.444599, -76.483682, -76.483276)
    #     grid_mode = ControlMode.LAWNMOWER
    #     all_waypoints = grid.get_waypoints(grid_mode)
    #     waypoints_to_visit = deque(all_waypoints)
    #     print(all_waypoints)
    #     unvisited_waypoints = r2d2.traverse_standard(
    #         waypoints_to_visit, allowed_dist_error, database)
    #     self.assertEqual(deque([]), unvisited_waypoints)

    # def test_different_heading(self):
    #     r2d2_state = Robot_State(xpos=42, ypos=-76, heading=math.pi,
    #                              epsilon=0.2, max_velocity=0.5, radius=0.2, phase=2, avoid_obstacle=False)
    #     r2d2 = Robot(robot_state=r2d2_state)
    #     database = DataBase(r2d2)

    #     allowed_dist_error = 0.5
    #     grid = Grid(42.444250, 42.444599, -76.483682, -76.483276)
    #     grid_mode = ControlMode.LAWNMOWER
    #     all_waypoints = grid.get_waypoints(grid_mode)
    #     waypoints_to_visit = deque(all_waypoints)
    #     unvisited_waypoints = r2d2.traverse_standard(
    #         waypoints_to_visit, allowed_dist_error, database)
    #     self.assertEqual(deque([]), unvisited_waypoints)

    # def test_with_minimal_noise(self):
    #     r2d2_state = Robot_State(xpos=42, ypos=-76, heading=math.pi, epsilon=0.2,
    #                              max_velocity=0.5, radius=0.2, phase=2, position_noise=0.05, avoid_obstacle=False)
    #     r2d2 = Robot(robot_state=r2d2_state)
    #     database = DataBase(r2d2)

    #     allowed_dist_error = 0.5
    #     grid = Grid(42.444250, 42.444599, -76.483682, -76.483276)
    #     grid_mode = ControlMode.LAWNMOWER
    #     all_waypoints = grid.get_waypoints(grid_mode)
    #     waypoints_to_visit = deque(all_waypoints)
    #     unvisited_waypoints = r2d2.traverse_standard(
    #         waypoints_to_visit, allowed_dist_error, database)
    #     self.assertEqual(deque([]), unvisited_waypoints)


if __name__ == '__main__':
    unittest.main()
