from engine.robot import Robot
# from electrical.rf_module import Device, RadioSession
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
    #test device initialization
    def test_device_init(self): 
        pass
        # robot_device = Device(0, '/dev/ttyS0')
        # basestation_device = Device(1, '/dev/ttyS0') #change the port
        # radio_session = RadioSession(self.robot_device)
        # self.assertEqual(robot_device.connected, False)
        # self.assertEqual(robot_device.device, 0)
        # self.assertEqual(basestation_device.connected, False)
        # self.assertEqual(basestation_device.connected, 1)
        # self.assertEqual(radio_session.device, robot_device)

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

        def setup():
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

        self.assertEqual(True, setup())

    #test_imu_setup contains simulated gps data
    def test_imu_setup(self):
        pass 

    #test_radio_session_setup contains simulated serial data
    def test_radio_session_setup(self):
        pass

# class TestNodes(unittest.TestCase):
#     # set up parameters for robot
#     x_pos = 5
#     y_pos = 6
#     heading = math.pi
#     init_mode = 'collect'
#     is_sim = True
#     init_charge = 100
#     init_capacity = 100

#     # deep copies because each test case changes the robot object,
#     # and for some reason the tests are executed from bottom to top
#     robot_one = Robot(x_pos, y_pos, heading, init_mode, \
#                       is_sim, init_charge, init_capacity)
#     robot_two = copy.deepcopy(robot_one)
#     robot_three = copy.deepcopy(robot_one)
#     robot_four = copy.deepcopy(robot_one)
#     robot_five = copy.deepcopy(robot_one)
#     robot_six = copy.deepcopy(robot_one)

#     distance = 10
#     turn_angle = math.pi / 2

#     def test_travel(self):
#         self.robot_one.travel(self.distance, self.turn_angle)
#         # Values calculated by hand based on kinematic equations
#         new_x = round(-1.3662, 3)
#         new_y = round(-0.366198, 3)
#         new_theta = round(3 * math.pi / 2, 3)
#         self.assertEqual([[float(new_x)], [float(new_y)], [float(new_theta)]], self.robot_one.state.tolist())

#     def test_move_forward_default(self):
#         # Values calculated by hand based on kinematic equations
#         self.robot_two.move_forward(self.distance)
#         new_x = -5
#         new_y = 5
#         new_theta = float(self.robot_two.state[2])
#         self.assertEqual([[float(new_x)], [float(new_y)], [float(new_theta)]], self.robot_two.state.tolist())

#     def test_move_forward_with_time(self):
#         # Values calculated by hand based on kinematic equations
#         self.robot_three.move_forward(self.distance, 5)
#         new_x = -45
#         new_y = 5
#         new_theta = float(self.robot_two.state[2])
#         self.assertEqual([[float(new_x)], [float(new_y)], [float(new_theta)]], self.robot_three.state.tolist())

#     def test_turn(self):
#         self.robot_four.turn(math.pi / 2)
#         self.assertEqual([4.712], self.robot_four.state[2])

#     def test_circle_turn(self):
#         original_angle = self.robot_five.state[2]
#         self.robot_five.turn(math.pi * 2)
#         self.assertEqual(original_angle, self.robot_five.state[2])

#     def test_turn_with_time(self):
#         self.robot_six.turn(math.pi / 2, 5)
#         self.assertEqual([4.712], self.robot_six.state[2])


if __name__ == '__main__':
    unittest.main()
