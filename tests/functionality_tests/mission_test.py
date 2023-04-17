# THIS IS A TESTING SCRIPT. USE THIS SCRIPT WHEN PHYSICALLY TESTING THE ROBOT AND CHANGE THE run() FUNCTION BY ADDING THE MISSION STATE YOU WANT TO TEST
# Obstacle Avoidance thread disabled in setup() so we dont accidentally test obstacle avoidance when we just want to test traversal, etc in isolation

from engine.robot_logic.robot_initialization import Robot

from engine.robot_logic.set_up import execute_setup
from engine.robot_logic.traversal import execute_traversal
from engine.robot_logic.obstacle_avoidance import execute_avoid_obstacle
from engine.robot_logic.docking import execute_docking
from engine.robot_logic.returning import execute_return

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
        execute_setup(self.r2d2)
        # disabled obstacle avoidance thread because if there is an obstacle,
        # the phase would go into obstacle avoidance despite wanting to solely test another phase (like traversal).
        self.rb_state.track_obstacle_thread.stop()

    def test_traverse(self):
        execute_traversal(self.r2d2, self.mission.waypoints_to_visit, self.mission.allowed_dist_error, self.mission.base_station_loc,
                                    self.mission.control_mode, self.mission.time_limit, self.mission.roomba_radius, self.database)

    def test_avoid_obstacle(self):
        # can only test avoid obstacle by doing one of the other phase and having an obstacle
        self.rb_state.track_obstacle_thread.start()
        execute_avoid_obstacle(self.r2d2, self.mission.waypoints_to_visit, self.mission.allowed_dist_error, self.mission.base_station_loc,
                                    self.mission.control_mode, self.mission.time_limit, self.mission.roomba_radius, self.database)

    def test_return(self):
        execute_return(self.r2d2, self.mission.base_station_loc, self.mission.base_station_angle, self.mission.allowed_docking_pos_error,
                                          self.mission.allowed_heading_error, self.database)

    def test_docking(self):
        execute_docking(self.r2d2)

    def test_complete(self):
        self.rb_state.phase = Phase.COMPLETE
        self.mission.execute_mission()

    def run(self):
        self.test_setup()


test = MissionTest()
test.run
