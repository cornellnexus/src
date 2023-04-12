from engine.robot import Phase

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
        while self.mission_state.robot.robot_state.phase != Phase.COMPLETE:
            if self.mission_state.robot.robot_state.phase == Phase.SETUP:
                # TODO: Determine if sensors are part of mission vs robot
                self.mission_state.robot.execute_setup(self.mission_state.robot_radio_session, self.mission_state.gps, self.mission_state.imu, self.mission_state.motor_controller)

            elif self.mission_state.robot.robot_state.phase == Phase.TRAVERSE:
                self.mission_state.waypoints_to_visit = self.mission_state.robot.execute_traversal(self.mission_state.waypoints_to_visit,
                                                                       self.mission_state.allowed_dist_error, self.mission_state.base_station.position,
                                                                       self.mission_state.control_mode, self.mission_state.time_limit,
                                                                       self.mission_state.roomba_radius, database)

            elif self.mission_state.robot.robot_state.phase == Phase.AVOID_OBSTACLE:
                self.mission_state.robot.execute_avoid_obstacle(self.mission_state.robot.robot_state.dist_to_goal, database)
                # add goal_loc param if goal_loc becomes dynamic

            elif self.mission_state.robot.robot_state.phase == Phase.RETURN:
                self.mission_state.robot.execute_return(self.mission_state.base_station.position, self.mission_state.base_station.heading,
                                          self.mission_state.allowed_docking_pos_error, self.mission_state.allowed_heading_error, database)

            elif self.mission_state.robot.robot_state.phase == Phase.DOCKING:
                self.mission_state.robot.execute_docking()
            
            #update the database with the most recent state
            database.update_data("phase", self.mission_state.robot.robot_state.phase)
            


