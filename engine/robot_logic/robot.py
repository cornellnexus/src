from engine.pid_controller import PID
from constants.definitions import *
from csv_files.csv_util import write_phase_to_csv

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

        self.loc_pid_x = PID(
            Kp=self.robot_state.position_kp, Ki=self.robot_state.position_ki, Kd=self.robot_state.position_kd, 
            target=0, sample_time=self.robot_state.time_step, output_limits=(None, None))

        self.loc_pid_y = PID(
            Kp=self.robot_state.position_kp, Ki=self.robot_state.position_ki, Kd=self.robot_state.position_kd, 
            target=0, sample_time=self.robot_state.time_step, output_limits=(None, None))

        self.head_pid = PID(
            Kp=self.robot_state.heading_kp, Ki=self.robot_state.heading_ki, Kd=self.robot_state.heading_kd, 
            target=0, sample_time=self.robot_state.time_step, output_limits=(None, None))

        # Write robot's phase to CSV file
        try:
            write_phase_to_csv(self.robot_state.phase)
        finally:
            pass



