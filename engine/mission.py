from engine.phase import Phase
from engine.robot_logic.set_up import execute_setup
from engine.robot_logic.traversal import execute_traversal
from engine.robot_logic.obstacle_avoidance import execute_avoid_obstacle
from engine.robot_logic.returning import execute_return
from engine.robot_logic.docking import execute_docking

class Mission:
    def __init__(self, mission_state):
        """
        Arguments:
            mission_state: an instance encapsulating conditions, measurements, etc. (i.e. all data) about this mission  
        """
        self.mission_state = mission_state


    def execute_mission(self, database):
        """
        Activates the main control loop. Depending on the robot's phase, different motion control algorithms are
        activated.
        """
        #add goal_loc param if goal_loc becomes dynamic
        #TODO: Determine if sensors are part of mission vs robot
        while self.mission_state.robot.robot_state.phase != Phase.COMPLETE: 
            phase = self.mission_state.robot.robot_state.phase
            if phase == Phase.SETUP: 
                execute_setup(self.mission_state.robot)
            elif phase == Phase.TRAVERSE:
                # TODO: clean up parameters for this function
                # allowed_dist_error, roomba_radius, and time_limit should be constants?
                self.mission_state.robot, self.mission_state.waypoints_to_visit = execute_traversal(self.mission_state, database)
            elif phase == Phase.AVOID_OBSTACLE: 
                execute_avoid_obstacle(self.mission_state.robot, 
                                       self.mission_state.robot.robot_state.dist_to_goal, database)
            elif phase == Phase.RETURN:
                execute_return(self.mission_state, database)
            elif phase == Phase.DOCKING:
                execute_docking(self.mission_state.robot)

             #update the database with the most recent state
            database.update_data("phase", self.mission_state.robot.robot_state.phase)
