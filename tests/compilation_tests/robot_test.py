from engine.robot import Robot
from electrical.radio_module import Device, RadioSession
from electrical.motor_controller import BasicMotorController, MotorController
# from electrical.gps import GPS
# from electrical.imu import IMU
import copy
import math
import unittest

'''
Unit tests for robot.py
NOTE: MANY OF THESE FUNCTIONS ARE BASED SOLELY ON KINEMATIC EQUATIONS
        for more comprehensive testing, we should make sure these kinematic
        equations are correct!
'''

class TestSetup(unittest.TestCase):
    #test_gps_setup contains simulated gps data    
    def test_gps_setup_true(self): 
        sim_data = [{"long": 0.0, "lat": 0.0}, {"long": 0.0, "lat": 0.0}, {"long": 0.0, "lat": 0.0}, {"long": 0.0, "lat": 0.0},
        {"long": 0.0, "lat": 0.0}, {"long": 0.0, "lat": 0.0}, {"long": 5.0, "lat": 5.0}, {"long": 5.0, "lat": 5.0}, 
        {"long": 5.0, "lat": 5.0}, {"long": 5.0, "lat": 5.0}, {"long": 5.0, "lat": 5.0}, {"long": 5.0, "lat": 5.0}, 
        {"long": 5.0, "lat": 5.0}, {"long": 5.0, "lat": 5.0}, {"long": 5.0, "lat": 5.0}, {"long": 5.0, "lat": 5.0}, 
        {"long": 5.0, "lat": 5.0}, {"long": 5.0, "lat": 5.0}, {"long": 5.0, "lat": 5.0}, {"long": 5.0, "lat": 5.0}, 
        {"long": 5.0, "lat": 5.0}, {"long": 5.0, "lat": 5.0}, {"long": 5.0, "lat": 5.0}, {"long": 5.0, "lat": 5.0}, 
        {"long": 5.0, "lat": 5.0}, {"long": 5.0, "lat": 5.0}, {"long": 5.0, "lat": 5.0}, {"long": 5.0, "lat": 5.0}, 
        {"long": 10.0, "lat": 10.0}, {"long": 10.0, "lat": 10.0}, {"long": 10.0, "lat": 10.0}, {"long": 10.0, "lat": 10.0}]

        def gps_setup():
            count = 0 
            gps_data = []
            while (len(gps_data) < 25): 
                count = count + 1 
                data = sim_data[count]
                if (data.get("long") != 0 and data.get("lat") != 0): 
                    gps_data.append(data)
                if (count >= 50): 
                    return False 
            return True

        self.assertEqual(True, gps_setup())

    #test_imu_setup contains simulated gps data
    def test_imu_setup(self):
        sim_data = [{"acc":0, "mag":0, "gyro":0}, {"acc":0, "mag":0, "gyro":0}, {"acc":0, "mag":0, "gyro":0},
        {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10}, 
        {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10}, 
        {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10},
        {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10},
        {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10},
        {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10},
        {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10},
        {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10},
        {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10}, {"acc":10, "mag":10, "gyro":10}
        ] 
        
        def imu_setup():
            imu_data = []
            count = 0
            while (len(imu_data) < 25): 
                count += 1 
                data = sim_data[count]
                if (data.get("acc") != 0 and data.get("mag") != 0 and data.get("gyro")!=0): 
                    imu_data.append(data)
                if (count > 50): 
                    return False
            return True 

        self.assertEqual(True, imu_setup())
        

    #test_radio_session_setup for device and radio session initialization
    #note: actual radio connection testing must be performed with hardware
    def test_radio_session_setup(self):
        robot_device = Device(None, 0) 
        base_station_device = Device(None, 1)
        robot_radio_session = RadioSession(robot_device)
        self.assertEqual(robot_radio_session.device.connected, False)
        self.assertEqual(robot_radio_session.device.device_number, 0)
        self.assertEqual(base_station_device.device_number, 1)

    #test motor_controller_setup calls the motor controller setup function, which will 
    #print a series of commands. Make sure they are in the order of: 
    #'go_forward', 'turn_left', 'turn_right', 'reverse', 'stop'
    def test_motor_controller_setup(self): 
        robot = Robot(x_pos = 0, y_pos = 0, heading = 0, epsilon = 0, max_v = 0, radius = 1, is_sim = True)
        motor_controller = BasicMotorController(robot)
        self.assertEqual(motor_controller.is_sim, True)
        motor_controller.setup()

class TestNodes(unittest.TestCase):
    # set up parameters for robot
    x_pos = 5
    y_pos = 6
    heading = math.pi
    init_mode = 'collect'
    is_sim = True
    init_charge = 100
    init_capacity = 100

    # deep copies because each test case changes the robot object,
    # and for some reason the tests are executed from bottom to top
    robot_one = Robot(x_pos, y_pos, heading, init_mode, \
                      is_sim, init_charge, init_capacity)
    robot_two = copy.deepcopy(robot_one)
    robot_three = copy.deepcopy(robot_one)
    robot_four = copy.deepcopy(robot_one)
    robot_five = copy.deepcopy(robot_one)
    robot_six = copy.deepcopy(robot_one)

    distance = 10
    turn_angle = math.pi / 2

    def test_travel(self):
        self.robot_one.travel(self.distance, self.turn_angle)
        # Values calculated by hand based on kinematic equations
        new_x = round(-1.3662, 3)
        new_y = round(-0.366198, 3)
        new_theta = round(3 * math.pi / 2, 3)
        self.assertEqual([[float(new_x)], [float(new_y)], [float(new_theta)]], self.robot_one.state.tolist())

    def test_move_forward_default(self):
        # Values calculated by hand based on kinematic equations
        self.robot_two.move_forward(self.distance)
        new_x = -5
        new_y = 5
        new_theta = float(self.robot_two.state[2])
        self.assertEqual([[float(new_x)], [float(new_y)], [float(new_theta)]], self.robot_two.state.tolist())

    def test_move_forward_with_time(self):
        # Values calculated by hand based on kinematic equations
        self.robot_three.move_forward(5)
        new_x = -45
        new_y = 5
        new_theta = float(self.robot_two.state[2])
        self.assertEqual([[float(new_x)], [float(new_y)], [float(new_theta)]], self.robot_three.state.tolist())

    def test_turn(self):
        self.robot_four.turn(math.pi / 2)
        self.assertEqual([4.712], self.robot_four.state[2])

    def test_circle_turn(self):
        original_angle = self.robot_five.state[2]
        self.robot_five.turn(math.pi * 2)
        self.assertEqual(original_angle, self.robot_five.state[2])

    def test_turn_with_time(self):
        self.robot_six.turn(math.pi / 2)
        self.assertEqual([4.712], self.robot_six.state[2])


if __name__ == '__main__':
    unittest.main()
