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
        self.rob = Robot(self.rb_state)
        self.rob_base = BaseStation(position=(42.444250, -76.483682), heading=math.pi/2, battery=None, plastic_load=None)
        self.database = DataBase(self.rob)
        self.m = Mission(robot=self.rob, base_station=self.rob_base, init_control_mode=ControlMode.LAWNMOWER)
    
    def test_setup(self):
        self.rob.execute_setup(self.m.robot_radio_session, self.m.gps, self.m.imu, self.m.motor_controller)

    def test_traverse(self):
        self.rob.execute_traversal(self.m.waypoints_to_visit, self.m.allowed_dist_error, self.m.base_station_loc, 
                                   self.m.control_mode, self.m.time_limit, self.m.roomba_radius, self.database)

    def test_avoid_obstacle(self):
        self.rob.execute_avoid_obstacle(self.m.robot.robot_state.dist_to_goal, self.database)

    def test_return(self):
        self.m.robot.execute_return(self.m.base_station_loc, self.m.base_station_angle, self.m.allowed_docking_pos_error, 
                                    self.m.allowed_heading_error, self.database)

    def test_docking(self):
        self.m.robot.execute_docking()

    def test_complete(self):
        self.rb_state.phase = Phase.COMPLETE
        self.m.execute_mission()

    def run(self):
        self.test_setup()


test = MissionTest()
test.run