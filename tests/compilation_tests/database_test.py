import unittest
import numpy as np

from engine.database import DataBase
from engine.robot import Robot
from engine.phase import Phase
from engine.robot_state import Robot_State

"""
Unit tests for database.py
"""


class TestDataBase(unittest.TestCase):
    # DataBase instances to test on
    robot_state_default = Robot_State(
        epsilon=0, max_velocity=0, radius=0, phase=Phase.SETUP
    )
    robot_default = Robot(robot_state=robot_state_default)

    db_default = DataBase(robot_default)

    # We can't make is_sim false because it fails GitHub merge tests.
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

    db_initial = DataBase(robot_initial)

    robot_state_one_param = Robot_State(
        epsilon=0,
        max_velocity=0,
        radius=0,
        phase=Phase.SETUP,
        plastic_level=3,
        battery=46,
        acceleration=[0.5, 0.2, 0.3],
    )
    robot_one_param = Robot(robot_state=robot_state_one_param)

    db_one_param = DataBase(robot_one_param)

    def test_str(self):
        db_str = (
            "phase: 1,\nstate [x, y, heading]: [0, 0, 0],\nis_sim: True,\nplastic_level: 0,\n"
            "battery: 100,\nmove_dist: 0.5,\nacceleration [x, y, z]: [0, 0, 0],\n"
            "magnetic_field [x, y, z]: [0, 0, 0],\ngyro_rotation [x, y, z]: [0, 0, 0],\n"
            "position_pid [proportional factor, integral factor, derivative factor]: [1, 0, 0],\n"
            "position_noise: 0,\n"
            "heading_pid [proportional factor, integral factor, derivative factor]: [1, 0, 0]"
        )

        db_initial_str = (
            "phase: 2,\nstate [x, y, heading]: [10, 20, 50],\nis_sim: True,\nplastic_level: 2,\n"
            "battery: 98,\nmove_dist: 0.6,\nacceleration [x, y, z]: [4.25, 3.2, 0.1],\n"
            "magnetic_field [x, y, z]: [0.1, 0.2, 0.3],\ngyro_rotation [x, y, z]: [0.5, 0.2, 0.6],\n"
            "position_pid [proportional factor, integral factor, derivative factor]: [0.12, 1.7, 9.2],\n"
            "position_noise: 0.23,\n"
            "heading_pid [proportional factor, integral factor, derivative factor]: [0.2, 0.4, 0.16]"
        )

        db_one_param_str = (
            "phase: 1,\nstate [x, y, heading]: [0, 0, 0],\nis_sim: True,\nplastic_level: 3,\n"
            "battery: 46,\nmove_dist: 0.5,\nacceleration [x, y, z]: [0.5, 0.2, 0.3],\n"
            "magnetic_field [x, y, z]: [0, 0, 0],\ngyro_rotation [x, y, z]: [0, 0, 0],\n"
            "position_pid [proportional factor, integral factor, derivative factor]: [1, 0, 0],\n"
            "position_noise: 0,\n"
            "heading_pid [proportional factor, integral factor, derivative factor]: [1, 0, 0]"
        )

        testcases = [
            (db_str, self.db_default),
            (db_initial_str, self.db_initial),
            (db_one_param_str, self.db_one_param),
        ]

        for expected_ans, database in testcases:
            self.assertEqual(expected_ans, str(database), str(database))

    def test_get_data(self):
        db_params = [
            (Phase.SETUP, "phase"),
            ([0, 0, 0], "state"),
            (True, "is_sim"),
            (0, "plastic_level"),
            (100, "battery"),
            (0.5, "move_dist"),
            ([0, 0, 0], "acceleration"),
            ([0, 0, 0], "magnetic_field"),
            ([0, 0, 0], "gyro_rotation"),
            ([1, 0, 0], "position_pid"),
            (0, "position_noise"),
            ([1, 0, 0], "heading_pid"),
        ]

        db_initial_params = [
            (Phase.TRAVERSE, "phase"),
            ([10, 20, 50], "state"),
            (True, "is_sim"),
            (2, "plastic_level"),
            (98, "battery"),
            (0.6, "move_dist"),
            ([4.25, 3.2, 0.1], "acceleration"),
            ([0.1, 0.2, 0.3], "magnetic_field"),
            ([0.5, 0.2, 0.6], "gyro_rotation"),
            ([0.12, 1.7, 9.2], "position_pid"),
            (0.23, "position_noise"),
            ([0.2, 0.4, 0.16], "heading_pid"),
        ]

        db_one_params = [
            (Phase.SETUP, "phase"),
            ([0, 0, 0], "state"),
            (True, "is_sim"),
            (3, "plastic_level"),
            (46, "battery"),
            (0.5, "move_dist"),
            ([0.5, 0.2, 0.3], "acceleration"),
            ([0, 0, 0], "magnetic_field"),
            ([0, 0, 0], "gyro_rotation"),
            ([1, 0, 0], "position_pid"),
            (0, "position_noise"),
            ([1, 0, 0], "heading_pid"),
        ]

        testcases = [
            (db_params, self.db_default),
            (db_initial_params, self.db_initial),
            (db_one_params, self.db_one_param),
        ]

        for expected_ans, database in testcases:
            for ans, name in expected_ans:
                multiple_params = [
                    "state",
                    "acceleration",
                    "magnetic_field",
                    "gyro_rotation",
                    "position_pid",
                    "heading_pid",
                ]
                if name in multiple_params:
                    data = database.get_data(name)
                    self.assertEqual(ans[0], data[0], name)
                    self.assertEqual(ans[1], data[1], name)
                    self.assertEqual(ans[2], data[2], name)
                else:
                    self.assertEqual(ans, database.get_data(name), name)

    def test_update_data(self):
        self.db_default.update_data("phase", Phase.SETUP)
        self.db_default.update_data("state", 3, 1, 20)
        self.db_default.update_data("state", y=5)

        self.db_initial.update_data("time_step", 0.7)
        self.db_initial.update_data("magnetic_field", 5, 6)
        self.db_initial.update_data("heading_pid", 9)
        self.db_initial.update_data("gyro_rotation", z=0.4)

        self.db_one_param.update_data("phase", Phase.DOCKING)
        self.db_one_param.update_data("phase", Phase.AVOID_OBSTACLE)
        self.db_one_param.update_data("acceleration", 0.1, 2.3)
        self.db_one_param.update_data("magnetic_field", 0.2, 0.15, 0.3)
        self.db_one_param.update_data("position_pid", y=0.5, z=0.82)

        db_new_params = [
            (Phase.SETUP, "phase"),
            ([3, 5, 20], "state"),
            (True, "is_sim"),
            (0, "plastic_level"),
            (100, "battery"),
            (0.5, "move_dist"),
            ([0, 0, 0], "acceleration"),
            ([0, 0, 0], "magnetic_field"),
            ([0, 0, 0], "gyro_rotation"),
            ([1, 0, 0], "position_pid"),
            (0, "position_noise"),
            ([1, 0, 0], "heading_pid"),
        ]

        db_initial_new_params = [
            (Phase.TRAVERSE, "phase"),
            ([10, 20, 50], "state"),
            (True, "is_sim"),
            (2, "plastic_level"),
            (98, "battery"),
            (0.6, "move_dist"),
            ([4.25, 3.2, 0.1], "acceleration"),
            ([5, 6, 0.3], "magnetic_field"),
            ([0.5, 0.2, 0.4], "gyro_rotation"),
            ([0.12, 1.7, 9.2], "position_pid"),
            (0.23, "position_noise"),
            ([9, 0.4, 0.16], "heading_pid"),
        ]

        db_one_new_params = [
            (Phase.AVOID_OBSTACLE, "phase"),
            ([0, 0, 0], "state"),
            (True, "is_sim"),
            (3, "plastic_level"),
            (46, "battery"),
            (0.5, "move_dist"),
            ([0.1, 2.3, 0.3], "acceleration"),
            ([0.2, 0.15, 0.3], "magnetic_field"),
            ([0, 0, 0], "gyro_rotation"),
            ([1, 0.5, 0.82], "position_pid"),
            (0, "position_noise"),
            ([1, 0, 0], "heading_pid"),
        ]

        testcases = [
            (db_new_params, self.db_default),
            (db_initial_new_params, self.db_initial),
            (db_one_new_params, self.db_one_param),
        ]

        for expected_ans, database in testcases:
            for ans, name in expected_ans:
                multiple_params = [
                    "state",
                    "acceleration",
                    "magnetic_field",
                    "gyro_rotation",
                    "position_pid",
                    "heading_pid",
                ]
                if name in multiple_params:
                    data = database.get_data(name)
                    self.assertEqual(ans[0], data[0], name)
                    self.assertEqual(ans[1], data[1], name)
                    self.assertEqual(ans[2], data[2], name)
                else:
                    self.assertEqual(ans, database.get_data(name), name)

    def test_phase_as_value(self):
        robot_state_phase = Robot_State(
            epsilon=0, max_velocity=0, radius=0, phase=Phase.SETUP
        )
        robot_phase = Robot(robot_state=robot_state_phase)
        db_robot_phase = DataBase(robot_phase)
        phases = list(Phase)
        num_phases = len(phases)
        for i in range(1, num_phases):
            self.assertEqual(i, db_robot_phase.phase_as_value())
            db_robot_phase.update_data("phase", phases[i])

    def test_make_packet(self):
        expected_outputs = [
            "phase:1;p_weight:00.0;acc:0.00,0.00,0.00;n_dist:00.0;rot:00.00;last_n:000.00,000.00;vel:0.00;next_n:000.00,000.00;coord:000.00,000.00,000.00;batt:100;ctrl:1",
            "phase:2;p_weight:02.0;acc:4.25,3.20,0.10;n_dist:00.0;rot:00.00;last_n:000.00,000.00;vel:0.00;next_n:000.00,000.00;coord:010.00,020.00,050.00;batt:098;ctrl:1",
            "phase:1;p_weight:03.0;acc:0.50,0.20,0.30;n_dist:00.0;rot:00.00;last_n:000.00,000.00;vel:0.00;next_n:000.00,000.00;coord:000.00,000.00,000.00;batt:046;ctrl:1",
        ]

        testcases = [
            (expected_outputs[0], self.db_default),
            (expected_outputs[1], self.db_initial),
            (expected_outputs[2], self.db_one_param),
        ]

        for expected_ans, database in testcases:
            self.assertEqual(expected_ans, database.make_packet())


if __name__ == "__main__":
    unittest.main()
