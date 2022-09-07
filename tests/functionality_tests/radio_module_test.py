from electrical.radio_module import RadioSession
import serial
import time


class RadioModuleTest: 
    """
    RadioModuleTest that tests the serial data transmission connection between two devices: robot 
    and base station during the setup phase.
    """
    
    def __init__(self, is_robot, is_basestation):
        self.is_robot = is_robot
        self.is_basestation = is_basestation
        
    def run(self): 
        stop_time = time.time() + 50
        if self.is_robot: 
            robot_serial = serial.Serial('/dev/ttyS0', 57600) #robot serial port number
            robot_radio = RadioSession(robot_serial)
            while time.time() < stop_time: 
                robot_radio.setup_robot()

        if self.is_basestation: 
            basestation_serial = serial.Serial('/dev/tty.usbserial-017543DC', 57600) #base serial port number
            basestation_radio = RadioSession(basestation_serial)
            while time.time() < stop_time:
                basestation_radio.setup_basestation()
                data = basestation_radio.receive_data()
                


basestation_radio = RadioModuleTest(False, True)
basestation_radio.run()