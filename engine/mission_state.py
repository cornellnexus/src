
from collections import deque
from engine.control_mode import ControlMode
from engine.grid import Grid
from engine.base_station import BaseStation


class Mission_State:
    """
    Contains Mission-specific information.
    """
    def __init__(self, robot, base_station_coord, init_control_mode, **kwargs):
        """
        Arguments:
            robot: the Robot object linked to this Mission
            base_station_coord: the location of the base station in GPS coordinates in the form (latitude, longitude) grid.
            init_control_mode: the traversal mode linked to this Mission.
        Instance Attributes:
            robot: the Robot object linked to this Mission
            control_mode: the traversal mode linked to this Mission.
            grid: the Grid which the robot should traverse
            base_station: the BaseStation object linked to this Mission.
            all_waypoints: TODO
            active_waypoints: TODO
            inactive_waypoints: TODO
            waypoints_to_visit: TODO
            allowed_dist_error: the maximum distance in meters that the robot can be from a node for the robot to
                have "visited" that node
            allowed_heading_error: the maximum error in radians a robot can have to target heading while turning
                in place.
            allowed_docking_pos_error: the maximum distance in meters the robot can be from "ready to dock" position
                before it can start docking.
            time_limit: the maximum time the robot can execute roomba traversal mode
            roomba_radius: the maximum radius from the base station that the robot in roomba traversal mode can move

        Important: All the ports of the electrical classes (ie. Serial) need to be updated to the respective 
                    ports they are connected to on the computer running the code.
        """    
        self.robot = robot
        self.control_mode = ControlMode(init_control_mode)
        self.grid = kwargs.get("grid", Grid(42.444250, 42.444599, -76.483682, -76.483276)) # Default grid to eng quad
        self.base_station = BaseStation(coord=base_station_coord,grid=self.grid)
        self.all_waypoints = self.grid.get_waypoints(self.control_mode)
        self.active_waypoints = self.grid.get_active_waypoints_list()
        self.inactive_waypoints = self.grid.get_inactive_waypoints_list()
        self.waypoints_to_visit = deque(self.all_waypoints)
        self.allowed_dist_error = kwargs.get("allowed_dist_error", 0.5)
        self.allowed_heading_error = kwargs.get("allowed_heading_error", 0.1)
        self.allowed_docking_pos_error = kwargs.get("allowed_docking_pos_error", 0.1)
        self.time_limit = kwargs.get("time_limit",50000)
        self.roomba_radius = kwargs.get("roomba_radius", 20)