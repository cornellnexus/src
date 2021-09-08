import numpy as np
from engine.grid import Grid
from engine.robot import Phase


class Mission:
    def __init__(self, robot, grid=Grid(42.444250, 42.444599, -76.483682, -76.483276), grid_mode="borders",
                 allowed_dist_error=0.5):
        self.robot = robot
        self.grid = grid
        self.waypoints = self.grid.get_waypoints(grid_mode)
        # self.all_goals = np.array(self.grid.gps_waypoints)
        # self.unvisited_goals = np.array(self.grid.gps_waypoints)
        self.allowed_dist_error = allowed_dist_error

    def execute_mission(self):
        while self.robot.phase != Phase.COMPLETE:
            if self.robot.phase == Phase.SETUP:
                self.robot.execute_setup()
            elif self.robot.phase == Phase.TRAVERSE:
                self.unvisited_goals = self.robot.execute_traversal(self.waypoints, self.allowed_dist_error)
            elif self.robot.phase == Phase.AVOID_OBSTACLE:
                self.robot.execute_avoid_obstacle()
            elif self.robot.phase == Phase.RETURN:
                self.robot.execute_return()

