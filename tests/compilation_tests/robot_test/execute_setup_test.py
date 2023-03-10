import unittest

from engine.robot import Robot
from engine.robot_state import Robot_State
from electrical.radio_module import RadioModule
from electrical.motor_controller import BasicMotorController, MotorController


'''
Unit tests for robot.py execute_setup function
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
        robot_radio_session = RadioModule(None)
        self.assertEqual(robot_radio_session.connected, False)

    #test motor_controller_setup calls the motor controller setup function, which will 
    #print a series of commands. Make sure they are in the order of: 
    #'go_forward', 'turn_left', 'turn_right', 'reverse', 'stop'
    def test_motor_controller_setup(self): 
        robot_state = Robot_State(x_pos = 0, y_pos = 0, heading = 0, epsilon = 0, max_velocity = 0, radius = 1)
        robot = Robot(robot_state=robot_state)
        motor_controller = BasicMotorController(robot.robot_state.is_sim)
        self.assertEqual(motor_controller.is_sim, True)
        motor_controller.setup()


if __name__ == '__main__':
    unittest.main()