from sys import is_finalizing
import serial 
import time 

""" This module receives the serially transmitted data from the radio frequency (rf) module """
class Device: 
    def __init__(self, serial, device_number):
        """ 
            Device Numbers: 
            0: robot's radio
            1: base station radio
        """
        self.ser = serial
        self.connected = False
        self.device_number = device_number


class RadioSession:
    """
    RadioSession establishes the function calls needed between two devices.

    Communication package headings used when sending messages or parsing received messages:
    'sr': start (robot)
    'sb': start (base station)
    'dr': data (robot) 
    'db': data (base station) 
    """

    def __init__(self, device):
        self.device = device

    def receive_data(self):
        byte_data = self.ser.readline()
        return byte_data

    def transmit_data(self,data, packet_type):
        assert(isinstance(packet_type), str)
        cast_data = bytes((packet_type + str(data) + '\n'), encoding= 'utf-8')
        self.ser.write(cast_data)

    #implementation of 2-way handshake between the robot and basestation/gui for serial data transmission
    #setup function called on the robot
    def setup_robot(self): 
        if (self.device.device_number != 0): #self.device should be 0
            print("error, set device to 0")
        while (not self.device.connected): #while rpi not connected to base station
            self.transmit_data('sr')
            receive = self.receive_data()
            if (receive == 'sb'):
                self.device.connected = True

    #setup function called on the base station 
    def setup_basestation(self):
        if (self.device.device_number != 1): 
            print("error, set device to 1")
        while (not self.device.connected): #while rpi not connected to base station
            receive = self.receive_data()
            if (receive == 'sr'):
                self.transmit_data('sb') 
                self.device.connected = True