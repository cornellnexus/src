from electrical.rf_module import Device, RadioSession
import unittest


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
        robot_device = Device(None, 0) 
        base_station_device = Device(None, 1)
        robot_radio_session = RadioSession(robot_device)
        self.assertEqual(robot_radio_session.device.connected, False)
        self.assertEqual(robot_radio_session.device.device_number, 0)
        self.assertEqual(base_station_device.device_number, 1)

if __name__ == '__main__':
    unittest.main()