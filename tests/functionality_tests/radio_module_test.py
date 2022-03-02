from electrical.radio_module import Device, RadioSession
import serial
import time


class RadioModuleTest: 
    def run(self): 
        robot = True
        basestation = False
        if robot: 
            robot_serial = serial.Serial('/dev/ttyS0', 57600) #robot serial port number
            robot_device = Device(0, robot_serial) 
            robot_radio = RadioSession(robot_device)
            robot_radio.setup_robot()

       
        if basestation: 
            basestation_serial = serial.Serial('/dev/ttyS1', 57600) #base serial port number
            basestation_device = Device(1, basestation_serial) 
            basestation_radio = RadioSession(basestation_device)
            basestation_radio.setup_basestation()

        

        


    # def execute_setup(self, robot_device, radio_session, gps, imu, motor_controller):
    #         if (robot_device == 0): 
    #             gps_setup = gps.setup() 
    #             imu_setup = imu.setup()
    #             radio_session.setup_robot()
    #             motor_controller.setup(self.is_sim)
    #         else: 
    #             radio_session.setup_basestation()
            
    #         radio_connected = radio_session.device.connected 

class RadioModuleTest:
    """ 
    DOCUMENTATION
    """
    def __init__(self, robot):