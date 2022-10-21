import unittest
import math
from engine.robot import Robot
from engine.mission import Mission
from engine.base_station import BaseStation
from engine.mission import ControlMode
from constants.definitions import ROOT_DIR
import random


class TestObstacleTracking(unittest.TestCase):

    def populate_standard_obstacle_data(self):
        mock_data_path = ROOT_DIR + '/tests/functionality_tests/csv/ultrasonic_values.csv'
        mock_data_file = open(mock_data_path, "w")
        mock_data_file.truncate()
        mock_data_file.write(
            "(600, 600, 600, 600, 600, 'assuming a straight line between robot and goal with rectangular box centered in the middle')\n")
        mock_data_file.write("(600, 600, 600, 600, 600, 'going forward')\n")
        mock_data_file.write("(550, 600, 600, 600, 600, '')\n")
        mock_data_file.write("(500, 600, 600, 600, 600, '')\n")
        mock_data_file.write("(450, 600, 600, 600, 600, '')\n")
        mock_data_file.write("(400, 600, 600, 600, 600, '')\n")
        mock_data_file.write("(350, 600, 600, 600, 600, '')\n")
        mock_data_file.write("(300, 600, 600, 600, 600, '')\n")
        mock_data_file.write("(250, 600, 600, 600, 600, '')\n")
        mock_data_file.write("(200, 600, 600, 600, 600, '')\n")
        mock_data_file.write("(150, 600, 600, 600, 600, '')\n")
        mock_data_file.write("(100, 600, 600, 600, 600, '')\n")
        mock_data_file.write("(100, 550, 600, 600, 600, 'turns towards one side of obstacle')\n")
        mock_data_file.write("(150, 500, 600, 600, 600, '')\n")
        mock_data_file.write("(200, 450, 550, 600, 600, '')\n")
        mock_data_file.write("(250, 400, 500, 600, 600, '')\n")
        mock_data_file.write("(300, 350, 450, 600, 600, 'finish turning')\n")
        mock_data_file.write("(350, 400, 400, 600, 600, 'going forward')\n")
        mock_data_file.write("(350, 400, 400, 600, 600, '')\n")
        mock_data_file.write("(350, 400, 400, 600, 600, '')\n")
        mock_data_file.write("(350, 400, 400, 600, 600, '')\n")
        mock_data_file.write("(600, 400, 400, 600, 600, 'corner of obstacle no longer in view')\n")
        mock_data_file.write("(600, 400, 400, 600, 600, '')\n")
        mock_data_file.write("(600, 600, 400, 600, 600, 'left sensor of robot hanging over obstacle')\n")
        mock_data_file.write("(600, 550, 550, 600, 600, 'turning corner')\n")
        mock_data_file.write(
            "(700, 600, 500, 600, 600, 'erroneous ultrasonic value greater than max reliable sensor value')\n")
        mock_data_file.write("(600, 550, 550, 600, 600, '')\n")
        mock_data_file.write("(600, 600, 500, 600, 600, '')\n")
        mock_data_file.write("(500, 550, 550, 600, 600, '')\n")
        mock_data_file.write("(550, 600, 500, 600, 600, '')\n")
        mock_data_file.write("(450, 550, 550, 600, 600, '')\n")
        mock_data_file.write("(500, 600, 500, 600, 600, '')\n")
        mock_data_file.write("(400, 550, 550, 600, 600, '')\n")
        mock_data_file.write("(450, 600, 500, 600, 600, '')\n")
        mock_data_file.write("(350, 550, 550, 600, 600, 'finish turning')\n")
        mock_data_file.write("(350, 550, 550, 600, 600, 'going forward')\n")
        mock_data_file.write("(350, 550, 550, 600, 600, '')\n")
        mock_data_file.write("(350, 550, 550, 600, 600, '')\n")
        mock_data_file.write("(350, 550, 550, 600, 600, '')\n")
        mock_data_file.write("(350, 550, 550, 600, 600, '')\n")
        mock_data_file.write("(600, 550, 550, 600, 600, 'turning corner')\n")
        mock_data_file.write(
            "(600, 600, 500, 600, 600, 'obstacle on left side not sensed in front sensor (goal to the left); stop obstacle avoiding')\n")
        mock_data_file.write("(600, 600, 450, 600, 600, 'left sensor decreases because passing by corner')\n")
        mock_data_file.write("(600, 600, 400, 600, 600, '')\n")
        mock_data_file.write("(600, 600, 350, 600, 600, '')\n")
        mock_data_file.write("(600, 600, 400, 600, 600, 'corner passed')\n")
        mock_data_file.write("(600, 600, 450, 600, 600, '')\n")
        mock_data_file.write("(600, 600, 500, 600, 600, '')\n")
        mock_data_file.write("(600, 600, 550, 600, 600, '')\n")
        mock_data_file.write("(600, 600, 600, 600, 600, 'obstacle too far away')\n")
        mock_data_file.write("(600, 600, 600, 600, 600, '')\n")
        mock_data_file.write("(600, 600, 600, 600, 600, '')\n")
        mock_data_file.write("(600, 600, 600, 600, 600, '')\n")
        mock_data_file.write("(600, 600, 600, 600, 600, '')\n")
        mock_data_file.write("(600, 600, 600, 600, 600, '')\n")
        mock_data_file.write("(600, 600, 600, 600, 600, 'goal reached, sensor value ends')\n")
        mock_data_file.close()

    def populate_faulty_data(self):
        mock_data_path = ROOT_DIR + '/tests/functionality_tests/csv/ultrasonic_values.csv'
        mock_data_file = open(mock_data_path, "w")
        mock_data_file.truncate()
        mock_data_file.write("(-100, 600, 600, 600, 600, 'sensor value less than 0')\n")
        mock_data_file.close()

    def populate_random_data(self, front_sensors_values):
        mock_data_path = ROOT_DIR + '/tests/functionality_tests/csv/ultrasonic_values.csv'
        mock_data_file = open(mock_data_path, "w")
        mock_data_file.truncate()
        for i in range(len(front_sensors_values)):
            front_sensor_value = front_sensors_values[i]
            lf_value = random.random() * 600
            lb_value = random.random() * 600
            rf_value = random.random() * 600
            rb_value = random.random() * 600
            data = "(" + str(front_sensor_value) + ", " + str(lf_value) + ", " + str(lb_value) + ", " + str(
                rf_value) + ", " + str(rb_value) + ")"
            mock_data_file.write(data + "\n")
        mock_data_file.close()

    def approximate(self, threshold, val1, val2):
        return threshold > abs(val1 - val2)

    def test_init(self):
        self.populate_standard_obstacle_data()
        result_path = ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv'
        result_file = open(result_path, "w")
        result_file.truncate()
        result_file.close()
        r2d2 = Robot(0, 0, math.pi / 2, epsilon=0.2, max_v=0.5, radius=0.2, init_phase=2, goal_location=(0, 600),
                     width=785, front_ultrasonic=None, lb_ultrasonic=None, lf_ultrasonic=None, rb_ultrasonic=None,
                     rf_ultrasonic=None)
        r2d2.track_obstacle()
        result_file = open(ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv', "r")
        result_content = list(map(lambda string: string.rstrip('\n'), result_file.readlines()))
        result_file.close()
        expected = []
        for i in range(2):
            expected.append("Not Avoid")
        for i in range(19):
            expected.append("Avoid")
        for i in range(7):
            expected.append("Not Avoid")
        for i in range(12):
            expected.append("Avoid")
        for i in range(16):
            expected.append("Not Avoid")

        self.assertEqual(expected, result_content)

    def test_fault(self):
        self.populate_faulty_data()
        result_path = ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv'
        result_file = open(result_path, "w")
        result_file.truncate()
        result_file.close()
        r2d2 = Robot(0, 0, math.pi / 2, epsilon=0.2, max_v=0.5, radius=0.2, init_phase=2, goal_location=(0, 600),
                     width=785, front_ultrasonic=None, lb_ultrasonic=None, lf_ultrasonic=None, rb_ultrasonic=None,
                     rf_ultrasonic=None)
        r2d2.track_obstacle()
        result_file = open(ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv', "r")
        result_content = list(map(lambda string: string.rstrip('\n'), result_file.readlines()))
        result_file.close()
        expected = []
        for i in range(1):
            expected.append("Avoid")
            expected.append("Fault")

        self.assertEqual(expected, result_content)

    def test_min_value(self):
        num_test = 10
        front_sensor = list()
        for i in range(num_test):
            front_sensor.append(random.random() * 700)
        self.populate_random_data(front_sensor)
        result_path = ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv'
        result_file = open(result_path, "w")
        result_file.truncate()
        result_file.close()
        random_width = random.random() * 700
        r2d2 = Robot(0, 0, math.pi / 2, epsilon=0.2, max_v=0.5, radius=0.2, init_phase=2, goal_location=(0, 600),
                     width=random_width, front_ultrasonic=None, lb_ultrasonic=None, lf_ultrasonic=None,
                     rb_ultrasonic=None, rf_ultrasonic=None)
        r2d2.track_obstacle()
        result_file = open(ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv', "r")
        result_content = list(map(lambda string: string.rstrip('\n'), result_file.readlines()))
        result_file.close()
        expected = []
        for i in range(len(front_sensor)):
            curr_val = front_sensor[i]
            if curr_val < r2d2.detect_obstacle_range:
                expected.append("Avoid")
            else:
                expected.append("Not Avoid")

        self.assertEqual(expected, result_content)
        self.assertEqual \
            (r2d2.detect_obstacle_range,
             min(r2d2.max_sensor_range,
                 (random_width / 2) / (r2d2.measuring_angle_in_rad / 2) + r2d2.front_sensor_offset))

    def test_detection_range(self):
        wide_robot = Robot(0, 0, math.pi / 2, epsilon=0.2, max_v=0.5, radius=0.2, init_phase=2, goal_location=(0, 600),
                     width=800, front_ultrasonic=None, lb_ultrasonic=None, lf_ultrasonic=None,
                     rb_ultrasonic=None, rf_ultrasonic=None)
        skinny_robot = Robot(0, 0, math.pi / 2, epsilon=0.2, max_v=0.5, radius=0.2, init_phase=2, goal_location=(0, 600),
                     width=200, front_ultrasonic=None, lb_ultrasonic=None, lf_ultrasonic=None,
                     rb_ultrasonic=None, rf_ultrasonic=None)
        self.assertEqual(self.approximate(1, wide_robot.max_sensor_range, wide_robot.detect_obstacle_range), True)
        self.assertEqual(self.approximate(1, 152.79, skinny_robot.detect_obstacle_range), True)


if __name__ == '__main__':
    unittest.main()
