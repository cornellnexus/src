import numpy as np

from engine.database import DataBase
import unittest

from engine.robot import Phase, Robot

'''
Unit tests for database.py
'''

class TestDataBase(unittest.TestCase):
    # DataBase instances to test on
    robot_default = Robot(x_pos= 0, y_pos =0, heading =0, epsilon =0, max_v =0, 
    radius =0, is_sim=True)

    db_default = DataBase(robot_default)


    robot_initial = Robot(x_pos= 0, y_pos =0, heading =0, epsilon =0, max_v =0, 
    radius =0, is_sim=False, plastic_weight=2, move_dist=.6, position_noise=0.23)
    robot_initial.position_kp = 0.12
    robot_initial.position_ki = 1.7
    robot_initial.position_kd = 9.2
    robot_initial.heading_kp = 0.2
    robot_initial.heading_ki = 0.4
    robot_initial.heading_kd = 0.16
    robot_initial.phase = Phase.TRAVERSE
    robot_initial.state = np.array([[10], [20], [50]])
    robot_initial.battery = 98
    robot_initial.magnetic_field = [0.1, 0.2, 0.3]
    robot_initial.gyro_rotation = [0.5, 0.2, 0.6]
    robot_initial.acceleration = [4.25, 3.2, 0.1]

    db_initial = DataBase(robot_initial)


    robot_one_param = Robot(x_pos= 0, y_pos =0, heading =0, epsilon =0, 
    max_v =0, radius =0, plastic_weight=3)
    robot_one_param.battery = 46
    robot_one_param.acceleration = [0.5, 0.2, 0.3]

    db_one_param = DataBase(robot_one_param)

    def test_str(self):
        db_str = "phase: 1,\nstate [x, y, heading]: [0, 0, 0],\nis_sim: True,\nplastic_weight: 0,\n" \
                 "battery: 100,\nmove_dist: 0.5,\nacceleration [x, y, z]: [0, 0, 0],\n" \
                 "magnetic_field [x, y, z]: [0, 0, 0],\ngyro_rotation [x, y, z]: [0, 0, 0],\n" \
                 "position_pid [proportional factor, integral factor, derivative factor]: [1, 0, 0],\n" \
                 "position_noise: 0,\n" \
                 "heading_pid [proportional factor, integral factor, derivative factor]: [1, 0, 0]"

        db_initial_str = "phase: 2,\nstate [x, y, heading]: [10, 20, 50],\nis_sim: False,\nplastic_weight: 2,\n" \
                         "battery: 98,\nmove_dist: 0.6,\nacceleration [x, y, z]: [4.25, 3.2, 0.1],\n" \
                         "magnetic_field [x, y, z]: [0.1, 0.2, 0.3],\ngyro_rotation [x, y, z]: [0.5, 0.2, 0.6],\n" \
                         "position_pid [proportional factor, integral factor, derivative factor]: [0.12, 1.7, 9.2],\n" \
                         "position_noise: 0.23,\n" \
                         "heading_pid [proportional factor, integral factor, derivative factor]: [0.2, 0.4, 0.16]"

        db_one_param_str = "phase: 1,\nstate [x, y, heading]: [0, 0, 0],\nis_sim: True,\nplastic_weight: 3,\n" \
                           "battery: 46,\nmove_dist: 0.5,\nacceleration [x, y, z]: [0.5, 0.2, 0.3],\n" \
                           "magnetic_field [x, y, z]: [0, 0, 0],\ngyro_rotation [x, y, z]: [0, 0, 0],\n" \
                           "position_pid [proportional factor, integral factor, derivative factor]: [1, 0, 0],\n" \
                           "position_noise: 0,\n" \
                           "heading_pid [proportional factor, integral factor, derivative factor]: [1, 0, 0]"

        testcases = [(db_str, self.db_default), (db_initial_str, self.db_initial), (db_one_param_str, self.db_one_param)]


        for (expected_ans, database) in testcases:
            self.assertEqual(expected_ans, str(database), str(database))

    def test_get_data(self):

        db_params = [(Phase.SETUP, "phase"), ([0, 0, 0], "state"), (True, "is_sim"), (0, "plastic_weight"),
                     (100, "battery"), (0.5, "move_dist"), ([0, 0, 0], "acceleration"),
                     ([0, 0, 0], "magnetic_field"), ([0, 0, 0], "gyro_rotation"),
                     ([1, 0, 0], "position_pid"), (0, "position_noise"), ([1, 0, 0], "heading_pid")
                     ]

        db_initial_params = [(Phase.TRAVERSE, "phase"), ([10, 20, 50], "state"), (False, "is_sim"),
                             (2, "plastic_weight"),
                             (98, "battery"), (0.6, "move_dist"), ([4.25, 3.2, 0.1], "acceleration"),
                             ([0.1, 0.2, 0.3], "magnetic_field"), ([0.5, 0.2, 0.6], "gyro_rotation"),
                             ([0.12, 1.7, 9.2], "position_pid"), (0.23, "position_noise"),
                             ([0.2, 0.4, 0.16], "heading_pid")
                             ]

        db_one_params = [(Phase.SETUP, "phase"), ([0, 0, 0], "state"), (True, "is_sim"), (3, "plastic_weight"),
                         (46, "battery"), (0.5, "move_dist"), ([0.5, 0.2, 0.3], "acceleration"),
                         ([0, 0, 0], "magnetic_field"), ([0, 0, 0], "gyro_rotation"),
                         ([1, 0, 0], "position_pid"), (0, "position_noise"), ([1, 0, 0], "heading_pid")
                         ]

        testcases = [(db_params, self.db_default), (db_initial_params, self.db_initial), (db_one_params, self.db_one_param)]

        for expected_ans, database in testcases:
            for (ans, name) in expected_ans:
                multiple_params = ["state", "acceleration", "magnetic_field", "gyro_rotation", "position_pid", "heading_pid"]
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
        self.db_default.update_data("is_sim", False)

        self.db_initial.update_data("time_step", 0.7)
        self.db_initial.update_data("magnetic_field", 5, 6)
        self.db_initial.update_data("heading_pid", 9)
        self.db_initial.update_data("gyro_rotation", z=0.4)

        self.db_one_param.update_data("phase", Phase.DOCKING)
        self.db_one_param.update_data("phase", Phase.AVOID_OBSTACLE)
        self.db_one_param.update_data("acceleration", 0.1, 2.3)
        self.db_one_param.update_data("magnetic_field", 0.2, 0.15, 0.3)
        self.db_one_param.update_data("position_pid", y=0.5, z=0.82)

        db_new_params = [(Phase.SETUP, "phase"), ([3, 5, 20], "state"), (False, "is_sim"), (0, "plastic_weight"),
                         (100, "battery"), (0.5, "move_dist"), ([0, 0, 0], "acceleration"),
                         ([0, 0, 0], "magnetic_field"), ([0, 0, 0], "gyro_rotation"),
                         ([1, 0, 0], "position_pid"), (0, "position_noise"), ([1, 0, 0], "heading_pid")
                         ]

        db_initial_new_params = [(Phase.TRAVERSE, "phase"), ([10, 20, 50], "state"), (False, "is_sim"),
                                 (2, "plastic_weight"),
                                 (98, "battery"), (0.6, "move_dist"), ([4.25, 3.2, 0.1], "acceleration"),
                                 ([5, 6, 0.3], "magnetic_field"), ([0.5, 0.2, 0.4], "gyro_rotation"),
                                 ([0.12, 1.7, 9.2], "position_pid"), (0.23, "position_noise"),
                                 ([9, 0.4, 0.16], "heading_pid")
                                 ]

        db_one_new_params = [(Phase.AVOID_OBSTACLE, "phase"), ([0, 0, 0], "state"), (True, "is_sim"), (3, "plastic_weight"),
                             (46, "battery"), (0.5, "move_dist"), ([0.1, 2.3, 0.3], "acceleration"),
                             ([0.2, 0.15, 0.3], "magnetic_field"), ([0, 0, 0], "gyro_rotation"),
                             ([1, 0.5, 0.82], "position_pid"), (0, "position_noise"), ([1, 0, 0], "heading_pid")
                             ]

        testcases = [(db_new_params, self.db_default), (db_initial_new_params, self.db_initial),
                     (db_one_new_params, self.db_one_param)]

        for expected_ans, database in testcases:
            for (ans, name) in expected_ans:
                multiple_params = ["state", "acceleration", "magnetic_field", "gyro_rotation", "position_pid", "heading_pid"]
                if name in multiple_params:
                    data = database.get_data(name)
                    self.assertEqual(ans[0], data[0], name)
                    self.assertEqual(ans[1], data[1], name)
                    self.assertEqual(ans[2], data[2], name)
                else:
                    self.assertEqual(ans, database.get_data(name), name)


if __name__ == '__main__':
    unittest.main()
