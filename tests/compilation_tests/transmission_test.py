import unittest
import numpy as np

from engine.transmittor import Transmittor
from engine.robot import Robot
from engine.mission import Mission
from engine.phase import Phase
from engine.robot_state import Robot_State
from engine.mission_state import Mission_State
from engine.grid import Grid
from engine.base_station import BaseStation
from engine.control_mode import ControlMode
import math
import json

"""
Unit tests for transmission.py
"""


class TestTransmission(unittest.TestCase):
    # DataBase instances to test on
    robot_state_initial = Robot_State(
        epsilon=0,
        max_velocity=0,
        radius=0,
        plastic_level=2,
        move_dist=0.6,
        position_noise=0.23,
        position_kp=0.12,
        position_ki=1.7,
        position_kd=9.2,
        heading_kp=0.2,
        heading_ki=0.4,
        heading_kd=0.16,
        phase=Phase.TRAVERSE,
        state=np.array([[10], [20], [50]]),
        battery=98,
        magnetic_field=[0.1, 0.2, 0.3],
        gyro_rotation=[0.5, 0.2, 0.6],
        acceleration=[4.25, 3.2, 0.1],
        is_sim=True,
    )
    robot_initial = Robot(robot_state=robot_state_initial)

    longMin, longMax, latMin, latMax = -76.483682, -76.483276, 42.444250, 42.444599
    g = Grid(longMin, longMax, latMin, latMax)
    robot_initial_base = BaseStation(
        coord=(latMin, longMin),
        grid=g,
        heading=math.pi / 2,
        battery=None,
        plastic_load=None)
    mission_state_initial = Mission_State(
        robot=robot_initial,
        base_station_coord=robot_initial_base.coord,
        init_control_mode=ControlMode.LAWNMOWER,
    )
    transmittor = Transmittor(robot_state_initial, mission_state_initial)

    def test_make_packet(self):
        actual_output = json.loads(self.transmittor.rmi())
        actual_output.pop('timestamp', None)
        modified_actual_output = json.dumps(actual_output)
        expected_output = formatted_json = json.dumps({
        "sensors": {
            "gps": {"connected": False, "reading": [0, 0]},
            "imu": {"connected": False, "reading": None},
            "wheel_motors": {
                "duty_cycle": None,
                "linear_velocity": 0,
                "left_wheel_velocity": None,
                "right_wheel_velocity": None
            },
            "ultrasonic": {
                "front_uls": {"connected": False, "distance_to_object": None},
                "left1_uls": {"connected": False, "distance_to_object": None},
                "left2_uls": {"connected": False, "distance_to_object": None},
                "right1_uls": {"connected": False, "distance_to_object": None},
                "right2_uls": {"connected": False, "distance_to_object": None}
            }
        },
        "battery": {
            "battery_percent": 98,
            "time_until_recharge": "00:00:00",
            "low_power_mode": False
        },
        "metrics": {
            "goal_loc": {
                "global_coord": [0, 0],
                "local_coord": [0, 0],
                "robot_goal_dist": 0
            },
            "state": {
                "global_coord": [10, 20],
                "local_coord": [10, 20],
                "heading": 50
            },
            "next_node_coord": {
                "global_coord": [42.44425, -76.483682],
                "local_coord": [0.0, 0.0]
            },
            "phase": "TRAVERSE",
            "plastic_level": 2,
            "acc": [4.25, 3.2, 0.1],
            "control mode": 1,
            "eta_to_base": "00:00:00",
            "goal_nodes_completed": 0,
            "eta_complete": "00:00:00",
            "eta_next_node": "00:00:00"
        }
    })
        self.assertEqual(expected_output, modified_actual_output)
        print(expected_output)
if __name__ == "__main__":
    unittest.main()
