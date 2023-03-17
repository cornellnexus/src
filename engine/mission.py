from collections import deque
from engine.robot import Phase
from engine.control_mode import ControlMode
from engine.kinematics import get_vincenty_x, get_vincenty_y
from enum import Enum
from engine.grid import Grid



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
        while self.robot.robot_state.phase != Phase.COMPLETE:
            if self.robot.robot_state.phase == Phase.SETUP:
                # TODO: Determine if sensors are part of mission vs robot
                self.robot.execute_setup(self.mission_state.robot_radio_session, self.mission_state.gps, self.mission_state.imu, self.mission_state.motor_controller)

            elif self.robot.robot_state.phase == Phase.TRAVERSE:
                self.waypoints_to_visit = self.robot.execute_traversal(self.mission_state.waypoints_to_visit,
                                                                       self.mission_state.allowed_dist_error, self.mission_state.base_station.position,
                                                                       self.mission_state.control_mode, self.mission_state.time_limit,
                                                                       self.mission_state.roomba_radius, database)

            elif self.robot.robot_state.phase == Phase.AVOID_OBSTACLE:
                self.robot.execute_avoid_obstacle(self.robot.robot_state.dist_to_goal, database)
                # add goal_loc param if goal_loc becomes dynamic

            elif self.robot.robot_state.phase == Phase.RETURN:
                self.robot.execute_return(self.mission_state.base_station.position, self.mission_state.base_station.heading,
                                          self.allowed_docking_pos_error, self.allowed_heading_error, database)

            elif self.robot.robot_state.phase == Phase.DOCKING:
                self.robot.execute_docking()
            
            #update the database with the most recent state
            database.update_data("phase", self.robot.robot_state.phase)
            


