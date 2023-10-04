# THIS IS A TESTING SCRIPT. USE THIS SCRIPT WHEN PHYSICALLY TESTING THE ROBOT AND CHANGE THE run() FUNCTION BY ADDING THE MISSION STATE YOU WANT TO TEST
# Obstacle Avoidance thread disabled in setup() so we dont accidentally test obstacle avoidance when we just want to test traversal, etc in isolation

from engine.robot import Robot
from engine.grid import Grid
from engine.base_station import BaseStation
from engine.mission_state import Mission_State
from engine.mission import Mission
from engine.control_mode import ControlMode
from engine.database import DataBase
from engine.robot_state import Robot_State
from engine.phase import Phase

import math


class MissionTest():

    def __init__(self):
        self.r2d2_state = Robot_State(xpos=0, ypos=0, heading=math.pi / 4, epsilon=0.2, max_velocity=0.5, radius=0.2, phase = Phase.TRAVERSE)
        self.r2d2 = Robot(self.r2d2_state)
        
            # longMin, longMax, latMin, latMax = get_coord_inputs()
        longMin, longMax, latMin, latMax = -76.483682, -76.483276, 42.444250, 42.444599
        g = Grid(longMin, longMax, latMin, latMax)
        self.r2d2_base = BaseStation(coord=(latMin, longMin), grid=g, heading=math.pi/2, battery=None, plastic_load=None)
        self.database = DataBase(self.r2d2)
        self.mission_state = Mission_State(robot=self.r2d2, base_station_coord=self.r2d2_base.coord,
                               init_control_mode=ControlMode.LAWNMOWER)
        self.mission = Mission(self.mission_state)

    def test_setup(self):
        self.r2d2.execute_setup()
        # disabled obstacle avoidance thread because if there is an obstacle,
        # the phase would go into obstacle avoidance despite wanting to solely test another phase (like traversal).
        self.r2d2_state.enable_obstacle_avoidance = False

    def test_traverse(self):
        self.r2d2.execute_traversal(self.mission.waypoints_to_visit, self.mission.allowed_dist_error, self.mission.base_station_loc,
                                    self.mission.control_mode, self.mission.time_limit, self.mission.roomba_radius, self.database)

    def test_avoid_obstacle(self):
        # can only test avoid obstacle by doing one of the other phase and having an obstacle
        self.r2d2_state.enable_obstacle_avoidance = True
        self.r2d2.execute_avoid_obstacle(self.mission.waypoints_to_visit, self.mission.allowed_dist_error, self.mission.base_station_loc,
                                    self.mission.control_mode, self.mission.time_limit, self.mission.roomba_radius, self.database)

    def test_return(self):
        self.r2d2.execute_return(self.mission.base_station_loc, self.mission.base_station_angle, self.mission.allowed_docking_pos_error,
                                          self.mission.allowed_heading_error, self.database)

    def test_docking(self):
        self.r2d2.execute_docking()

    def test_complete(self):
        self.r2d2_state.phase = Phase.COMPLETE
        self.mission.execute_mission()

    def run(self):
        self.test_setup()


test = MissionTest()
test.run
