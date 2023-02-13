from engine.robot import Robot
from engine.grid import Grid
from engine.robot import Phase
from engine.base_station import BaseStation
from engine.mission import Mission
from engine.mission import ControlMode
from engine.database import DataBase

import math

class MissionTest():

    def __init__(self):
        self.rob = Robot(x_pos=0, y_pos=0, heading=0, epsilon=0.2, max_v=0.5, radius=1, is_sim=False, position_kp=1, position_ki=0,
                 position_kd=0, position_noise=0, heading_kp=1, heading_ki=0, heading_kd=0, heading_noise=0,
                 init_phase=1, time_step=1, move_dist=.5, turn_angle=0.5, plastic_weight=0)
        self.rob_base = BaseStation(position=(42.444250, -76.483682), heading=math.pi/2, battery=None, plastic_load=None)
        self.database = DataBase(self.rob)
        self.m = Mission(robot=self.rob, base_station=self.rob_base, init_control_mode=ControlMode.LAWNMOWER, grid=Grid(42.444250, 42.444599, -76.483682, -76.483276),
                 allowed_dist_error=0.5, allowed_heading_error=0.1, allowed_docking_pos_error=0.1,
                 time_limit=50000, roomba_radius=20)
                 
    
    def test_setup(self):
        self.rob.execute_setup(self.m.robot_radio_session, self.m.gps, self.m.imu, self.m.motor_controller)

    def test_traverse():
        pass

    def test_avoid_obstacle():
        pass

    def test_return():
        pass

    def test_docking():
        pass

    def test_complete():
        pass

    def run():
        pass

test = MissionTest()
# test.run