from collections import deque
from engine.grid import Grid
from engine.robot import Phase
from engine.kinematics import get_vincenty_x, get_vincenty_y
import math


class Mission:
    def __init__(self, robot, base_station, grid=Grid(42.444250, 42.444599, -76.483682, -76.483276),
                 grid_mode="lawn_border", allowed_dist_error=0.5, allowed_heading_error=0.1,
                 allowed_docking_pos_error=0.1):
        """
        Arguments:
            robot: the Robot object linked to this Mission
            base_station: the BaseStation object linked to this Mission.
            grid: the Grid which the robot should traverse
            grid_mode: "borders" if the grid's nodes should only include corner nodes, "full" if all nodes should be
                used
            allowed_dist_error: the maximum distance in meters that the robot can be from a node for the robot to
                have "visited" that node
            allowed_heading_error: the maximum error in radians a robot can have to target heading while turning
                in place.
            allowed_docking_pos_error: the maximum distance in meters the robot can be from "ready to dock" position
                before it can start docking.
        """
        self.robot = robot
        self.grid = grid
        self.all_waypoints = self.grid.get_waypoints(grid_mode)
        self.waypoints_to_visit = deque(self.all_waypoints)
        self.allowed_dist_error = allowed_dist_error
        self.allowed_heading_error = allowed_heading_error
        self.base_station_angle = base_station.heading
        self.allowed_docking_pos_error = allowed_docking_pos_error
        x = get_vincenty_x((grid.lat_min, grid.long_min), base_station.position)
        y = get_vincenty_y((grid.lat_min, grid.long_min), base_station.position)
        self.base_station_loc = (x, y)

    def execute_mission(self):
        """
        Activates the main control loop. Depending on the robot's phase, different motion control algorithms are
        activated.
        """
        while self.robot.phase != Phase.COMPLETE:
            if self.robot.phase == Phase.SETUP:
                self.robot.execute_setup()

            elif self.robot.phase == Phase.TRAVERSE:
                self.waypoints_to_visit = self.robot.execute_traversal(self.waypoints_to_visit,
                                                                       self.allowed_dist_error)

            elif self.robot.phase == Phase.AVOID_OBSTACLE:
                self.robot.execute_avoid_obstacle()

            elif self.robot.phase == Phase.RETURN:
                self.robot.execute_return(self.base_station_loc, self.base_station_angle,
                                          self.allowed_docking_pos_error, self.allowed_heading_error)

            elif self.robot.phase == Phase.DOCKING:
                self.robot.execute_docking()

