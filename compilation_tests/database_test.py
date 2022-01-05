from engine.database import DataBase
import unittest

'''
Unit tests for database.py
'''


class TestDataBase(unittest.TestCase):
    # DataBase instances to test on
    db = DataBase()

    db_initial = DataBase(2, [10, 20, 50], is_sim=False, plastic_weight=2, battery=98,
                          move_dist=.6, acceleration=[4.25, 3.2, 0.1], magnetic_field=[0.1, 0.2, 0.3],
                          gyro_rotation=[0.5, 0.2, 0.6],
                          position_pid=[0.12, 1.7, 9.2], position_noise=0.23, heading_pid=[0.2, 0.4, 0.16])

    db_one_param = DataBase(plastic_weight=3, battery=46, acceleration=[0.5, 0.2, 0.3])

    def test_str(self):
        db_str = "phase: 0,\nstate [x, y, heading]: [0, 0, 0],\nis_sim: True,\nplastic_weight: 0,\n" \
                 "battery: 0,\nmove_dist: 0.5,\nacceleration [x, y, z]: [0, 0, 0],\n" \
                 "magnetic_field [x, y, z]: [0, 0, 0],\ngyro_rotation [x, y, z]: [0, 0, 0],\n" \
                 "position_pid [proportional factor, integral factor, derivative factor]: [0, 0, 0],\n" \
                 "position_noise: 0,\n" \
                 "heading_pid [proportional factor, integral factor, derivative factor]: [0, 0, 0]"

        db_initial_str = "phase: 2,\nstate [x, y, heading]: [10, 20, 50],\nis_sim: False,\nplastic_weight: 2,\n" \
                         "battery: 98,\nmove_dist: 0.6,\nacceleration [x, y, z]: [4.25, 3.2, 0.1],\n" \
                         "magnetic_field [x, y, z]: [0.1, 0.2, 0.3],\ngyro_rotation [x, y, z]: [0.5, 0.2, 0.6],\n" \
                         "position_pid [proportional factor, integral factor, derivative factor]: [0.12, 1.7, 9.2],\n" \
                         "position_noise: 0.23,\n" \
                         "heading_pid [proportional factor, integral factor, derivative factor]: [0.2, 0.4, 0.16]"

        db_one_param_str = "phase: 0,\nstate [x, y, heading]: [0, 0, 0],\nis_sim: True,\nplastic_weight: 3,\n" \
                           "battery: 46,\nmove_dist: 0.5,\nacceleration [x, y, z]: [0.5, 0.2, 0.3],\n" \
                           "magnetic_field [x, y, z]: [0, 0, 0],\ngyro_rotation [x, y, z]: [0, 0, 0],\n" \
                           "position_pid [proportional factor, integral factor, derivative factor]: [0, 0, 0],\n" \
                           "position_noise: 0,\n" \
                           "heading_pid [proportional factor, integral factor, derivative factor]: [0, 0, 0]"

        testcases = [(db_str, self.db), (db_initial_str, self.db_initial), (db_one_param_str, self.db_one_param)]

        for (expected_ans, database) in testcases:
            self.assertEqual(expected_ans, str(database))

    def test_get_data(self):

        db_params = [(0, "phase"), ([0, 0, 0], "state"), (True, "is_sim"), (0, "plastic_weight"),
                     (0, "battery"), (0.5, "move_dist"), ([0, 0, 0], "acceleration"),
                     ([0, 0, 0], "magnetic_field"), ([0, 0, 0], "gyro_rotation"),
                     ([0, 0, 0], "position_pid"), (0, "position_noise"), ([0, 0, 0], "heading_pid")
                     ]

        db_initial_params = [(2, "phase"), ([10, 20, 50], "state"), (False, "is_sim"),
                             (2, "plastic_weight"),
                             (98, "battery"), (0.6, "move_dist"), ([4.25, 3.2, 0.1], "acceleration"),
                             ([0.1, 0.2, 0.3], "magnetic_field"), ([0.5, 0.2, 0.6], "gyro_rotation"),
                             ([0.12, 1.7, 9.2], "position_pid"), (0.23, "position_noise"),
                             ([0.2, 0.4, 0.16], "heading_pid")
                             ]

        db_one_params = [(0, "phase"), ([0, 0, 0], "state"), (True, "is_sim"), (3, "plastic_weight"),
                         (46, "battery"), (0.5, "move_dist"), ([0.5, 0.2, 0.3], "acceleration"),
                         ([0, 0, 0], "magnetic_field"), ([0, 0, 0], "gyro_rotation"),
                         ([0, 0, 0], "position_pid"), (0, "position_noise"), ([0, 0, 0], "heading_pid")
                         ]

        testcases = [(db_params, self.db), (db_initial_params, self.db_initial), (db_one_params, self.db_one_param)]

        for expected_ans, database in testcases:
            for (ans, name) in expected_ans:
                self.assertEqual(ans, database.get_data(name))

    def test_update_data(self):
        self.db.update_data("phase", 1)
        self.db.update_data("state", 3, 1, 20)
        self.db.update_data("state", y=5)
        self.db.update_data("is_sim", False)

        self.db_initial.update_data("time_step", 0.7)
        self.db_initial.update_data("magnetic_field", 5, 6)
        self.db_initial.update_data("heading_pid", 9)
        self.db_initial.update_data("gyro_rotation", z=0.4)

        self.db_one_param.update_data("phase", 5)
        self.db_one_param.update_data("phase", 3)
        self.db_one_param.update_data("acceleration", 0.1, 2.3)
        self.db_one_param.update_data("magnetic_field", 0.2, 0.15, 0.3)
        self.db_one_param.update_data("position_pid", y=0.5, z=0.82)

        db_new_params = [(1, "phase"), ([3, 5, 20], "state"), (False, "is_sim"), (0, "plastic_weight"),
                         (0, "battery"), (0.5, "move_dist"), ([0, 0, 0], "acceleration"),
                         ([0, 0, 0], "magnetic_field"), ([0, 0, 0], "gyro_rotation"),
                         ([0, 0, 0], "position_pid"), (0, "position_noise"), ([0, 0, 0], "heading_pid")
                         ]

        db_initial_new_params = [(2, "phase"), ([10, 20, 50], "state"), (False, "is_sim"),
                                 (2, "plastic_weight"),
                                 (98, "battery"), (0.6, "move_dist"), ([4.25, 3.2, 0.1], "acceleration"),
                                 ([5, 6, 0.3], "magnetic_field"), ([0.5, 0.2, 0.4], "gyro_rotation"),
                                 ([0.12, 1.7, 9.2], "position_pid"), (0.23, "position_noise"),
                                 ([9, 0.4, 0.16], "heading_pid")
                                 ]

        db_one_new_params = [(3, "phase"), ([0, 0, 0], "state"), (True, "is_sim"), (3, "plastic_weight"),
                             (46, "battery"), (0.5, "move_dist"), ([0.1, 2.3, 0.3], "acceleration"),
                             ([0.2, 0.15, 0.3], "magnetic_field"), ([0, 0, 0], "gyro_rotation"),
                             ([0, 0.5, 0.82], "position_pid"), (0, "position_noise"), ([0, 0, 0], "heading_pid")
                             ]

        testcases = [(db_new_params, self.db), (db_initial_new_params, self.db_initial),
                     (db_one_new_params, self.db_one_param)]

        for expected_ans, database in testcases:
            for (ans, name) in expected_ans:
                self.assertEqual(ans, database.get_data(name))


if __name__ == '__main__':
    unittest.main()
