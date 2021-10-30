from collections import deque
from engine.grid import Grid
from engine.robot import Phase
from electrical.rf_module import Device, RadioSession


class Mission:
    def __init__(self, robot, grid=Grid(42.444250, 42.444599, -76.483682, -76.483276), grid_mode="borders",
                 allowed_dist_error=0.5):
        """
        Arguments:
            robot: the Robot object linked to this Mission
            grid: the Grid which the robot should traverse
            grid_mode: "borders" if the grid's nodes should only include corner nodes, "full" if all nodes should be used
            allowed_dist_error: the maximum distance in meters that the robot can be from a node for the robot to have
                "visited" that node
        """
        self.robot = robot
        self.grid = grid
        self.all_waypoints = self.grid.get_waypoints(grid_mode)
        self.waypoints_to_visit = deque(self.all_waypoints)
        self.allowed_dist_error = allowed_dist_error
        self.rpi_device = Device(0, '/dev/ttyS0')
        self.base_device = Device(1, '/dev/ttyS0') #temp
        self.radio_session = RadioSession(self.rpi_device)

    def execute_mission(self):
        """
        Activates the main control loop. Depending on the robot's phase, different motion control algorithms are
        activated.
        """
        while self.robot.phase != Phase.COMPLETE:
            if self.robot.phase == Phase.SETUP:
                self.robot.execute_setup(self.radio_session)

            elif self.robot.phase == Phase.TRAVERSE:
                self.waypoints_to_visit = self.robot.execute_traversal(self.waypoints_to_visit,
                                                                       self.allowed_dist_error)

            elif self.robot.phase == Phase.AVOID_OBSTACLE:
                self.robot.execute_avoid_obstacle()

            elif self.robot.phase == Phase.RETURN:
                self.robot.execute_return()

