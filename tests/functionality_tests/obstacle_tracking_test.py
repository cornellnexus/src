import unittest
import math
from engine.robot import Robot
from engine.robot_state import Robot_State
from engine.mission import Mission
from engine.control_mode import ControlMode
from constants.definitions import ROOT_DIR
import random


class TestObstacleTracking(unittest.TestCase):

    def populate_standard_obstacle_data(self):
        mock_data_path = ROOT_DIR + '/tests/functionality_tests/csv/ultrasonic_values.csv'
        mock_data_file = open(mock_data_path, "w")
        mock_data_file.truncate()
        # csv data: front_sensor_value, lf_sensor_value, lb_sensor_value, rf_sensor_value, rb_sensor_value, comment
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

    def difference_in_threshold(self, threshold, val1, val2):
        return threshold > abs(val1 - val2)

    def test_init(self):
        self.populate_standard_obstacle_data()
        result_path = ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv'
        result_file = open(result_path, "w")
        result_file.truncate()
        result_file.close()
        r2d2_state = Robot_State(xpos=0, ypos=0, heading=math.pi/2, epsilon=0.2, max_velocity=0.5, radius=0.2, phase = 2, width = 785)
        r2d2 = Robot(r2d2_state)
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
        r2d2_state = Robot_State(xpos=0, ypos=0, heading=math.pi/2, epsilon=0.2, max_velocity=0.5, radius=0.2, phase = 2, width = 785)
        r2d2 = Robot(r2d2_state)
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
        r2d2_state = Robot_State(xpos=0, ypos=0, heading=math.pi/2, epsilon=0.2, max_velocity=0.5, radius=0.2, phase = 2, width = random_width)
        r2d2 = Robot(r2d2_state)
        r2d2.track_obstacle()
        result_file = open(ROOT_DIR + '/tests/functionality_tests/csv/avoid_obstacle_result.csv', "r")
        result_content = list(map(lambda string: string.rstrip('\n'), result_file.readlines()))
        result_file.close()
        expected = []
        for i in range(len(front_sensor)):
            curr_val = front_sensor[i]
            if curr_val < r2d2.robot_state.detect_obstacle_range:
                expected.append("Avoid")
            else:
                expected.append("Not Avoid")

        self.assertEqual(expected, result_content)
        self.assertEqual \
            (r2d2.robot_state.detect_obstacle_range,
             min(r2d2.robot_state.max_sensor_range,
                 ((r2d2.robot_state.width + r2d2.robot_state.width_margin) / 2) /
                 math.cos(math.radians((180 - r2d2.robot_state.sensor_measuring_angle) / 2)) + r2d2.robot_state.front_sensor_offset))

    def test_detection_range(self):
        wide_robot_state = Robot_State(xpos=0, ypos=0, heading=math.pi/2, epsilon=0.2, max_velocity=0.5, radius=0.2, phase = 2, width = 800)
        wide_robot = Robot(wide_robot_state)
        skinny_robot_state = Robot_State(xpos=0, ypos=0, heading=math.pi/2, epsilon=0.2, max_velocity=0.5, radius=0.2,phase = 2, width = 200)
        skinny_robot = Robot(skinny_robot_state)
        self.assertEqual(self.difference_in_threshold(1, wide_robot.robot_state.max_sensor_range, wide_robot.robot_state.detect_obstacle_range), True)
        self.assertEqual(self.difference_in_threshold(1, 165.1, skinny_robot.robot_state.detect_obstacle_range), True)
        self.assertEqual(self.difference_in_threshold(0, 165.1, skinny_robot.robot_state.detect_obstacle_range), False)
        self.assertEqual(self.difference_in_threshold(1, 200, skinny_robot.robot_state.detect_obstacle_range), False)



if __name__ == '__main__':
    unittest.main()
