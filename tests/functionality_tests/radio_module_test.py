from electrical.radio_module import Device, RadioSession
import serial
import time


class RadioModuleTest: 
    """
    RadioModuleTest that tests the serial data transmission connection between two devices: robot 
    and base station during the setup phase.
    """
    def run(self, is_robot, is_basestation): 
        stop_time = time.time() + 50
        if is_robot: 
            robot_serial = serial.Serial('/dev/ttyS0', 57600) #robot serial port number
            robot_device = Device(0, robot_serial) 
            robot_radio = RadioSession(robot_device)
            while time.time() < stop_time: 
                robot_radio.setup_robot()

        if is_basestation: 
            basestation_serial = serial.Serial('/dev/ttyS1', 57600) #base serial port number
            basestation_device = Device(1, basestation_serial) 
            basestation_radio = RadioSession(basestation_device)
            while time.time() < stop_time: 
                basestation_radio.setup_basestation()


robot_radio = RadioModuleTest(True, False)
basestation_radio = RadioModuleTest(False, True) 