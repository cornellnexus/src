from engine.pid_controller import PID
from constants.definitions import *
from csv_files.csv_util import write_phase_to_csv

from engine.robot_logic.set_up import setup_logic
from engine.robot_logic.traversal import traversal_logic
from engine.robot_logic.obstacle_avoidance import avoid_obstacle_logic
from engine.robot_logic.returning import return_logic
from engine.robot_logic.docking import docking_logic


class Robot:
    """
    A class whose objects contain robot-specific information, and methods to execute individual phases.

    Initializes the robot with the given position and heading.
    Parameters:
    state = Robot's state, np.array
        state contains the robot's x position, y position, and heading
    phase = 'collect' when traversing through grid, 'return' when returning to
        base station
    is_sim = False when running code with physical robot, True otherwise
    Preconditions:
    position is list of size 2
    heading is in range [0..359]
    """

    def __init__(self, robot_state):
        """
        Arguments:
        robot_state: an instance encapsulating conditions, measurements, etc. (i.e. all data)
                     about this robot
        """
        self.robot_state = robot_state

        # Write robot's phase to CSV file
        try:
            write_phase_to_csv(self.robot_state.phase)
        finally:
            pass

    def execute_setup(self):
        setup_logic(self.robot_state)

    def execute_traversal(self, mission_state, database):
        return traversal_logic(self.robot_state, mission_state, database)

    def execute_obstacle_avoidance(
        self,
    ):
        avoid_obstacle_logic(self.robot_state)

    def execute_return(self, mission_state, database):
        return_logic(self.robot_state, mission_state, database)

    def execute_docking(self):
        docking_logic(self.robot_state)
