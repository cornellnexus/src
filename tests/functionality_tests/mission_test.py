# THIS IS A TESTING SCRIPT. USE THIS SCRIPT WHEN PHYSICALLY TESTING THE ROBOT AND CHANGE THE run() FUNCTION BY ADDING THE MISSION STATE YOU WANT TO TEST

from engine.robot import Robot
from engine.base_station import BaseStation
from engine.mission import Mission
from engine.mission import ControlMode
from engine.database import DataBase
from engine.robot_state import Robot_State
from engine.phase import Phase

import math


class MissionTest():

    def __init__(self):
        self.rb_state = Robot_State(0, 0, 0, .2, .5, .2)
        self.r2d2 = Robot(self.rb_state)
        self.r2d2_base = BaseStation(position=(
            42.444250, -76.483682), heading=math.pi/2, battery=None, plastic_load=None)
        self.database = DataBase(self.r2d2)
        self.mission = Mission(robot=self.r2d2, base_station=self.r2d2_base,
                               init_control_mode=ControlMode.LAWNMOWER)

    def test_setup(self):
        self.r2d2.execute_setup(self.mission.robot_radio_session,
                                self.mission.gps, self.mission.imu, self.mission.motor_controller)
        self.rb_state.track_obstacle_thread.stop()

    def test_traverse(self):
        self.r2d2.execute_traversal(self.mission.waypoints_to_visit, self.mission.allowed_dist_error, self.mission.base_station_loc,
                                    self.mission.control_mode, self.mission.time_limit, self.mission.roomba_radius, self.database)

    def test_avoid_obstacle(self):
        # can only test avoid obstacle by doing one of the other phase and having an obstacle
        self.rb_state.track_obstacle_thread.start()
        self.r2d2.execute_traversal(self.mission.waypoints_to_visit, self.mission.allowed_dist_error, self.mission.base_station_loc,
                                    self.mission.control_mode, self.mission.time_limit, self.mission.roomba_radius, self.database)

    def test_return(self):
        self.mission.robot.execute_return(self.mission.base_station_loc, self.mission.base_station_angle, self.mission.allowed_docking_pos_error,
                                          self.mission.allowed_heading_error, self.database)

    def test_docking(self):
        self.mission.robot.execute_docking()

    def test_complete(self):
        self.rb_state.phase = Phase.COMPLETE
        self.mission.execute_mission()

    def run(self):
        self.test_setup()


test = MissionTest()
test.run
