from sys import is_finalizing
import serial 
import time 

""" This module receives the serially transmitted data from the rf module"""
class Device: 
    def __init__(self, device_number, port):
        self.ser = serial.Serial(port, 57600)
        self.connected = False
        self.device = device_number
class RadioSession:
    def __init__(self, device):
        #'sr': start (rpi)
        #'sb': start (base station)
        #'dr': data (rpi) 
        #'db': data (base station)
        self.packet_type = ['sr', 'sb', 'dr', 'db']
        self.device = device 
        self.device_list = [0, 1] #0 is rpi, 1 is base station

    def receive_data(self):
        byte_data = self.ser.readline()
        return byte_data

    def transmit_data(self,data, packet_type):
        assert(isinstance(packet_type), str)
        cast_data = bytes((packet_type + str(data) + '\n'), encoding= 'utf-8')
        self.ser.write(cast_data)

    #implementation of 2-way handshake
    #raspberry pi serves as "client" in 2-way handshake
    #returns True if Rpi successfully set up 
    def startup_robot(self): 
        if (self.device != 0): #self.device should be 0
            return "error, set device to 0" 
        while (not self.connect): #while rpi not connected to base station
            self.transmit_data('sr')
            receive = self.receive_data()
            if (receive == 'sb'):
                self.connect = True

    def startup_basestation(self):
        if (self.device != 1): 
            return "error, set device to 1"
        while (not self.connect): #while rpi not connected to base station
            receive = self.receive_data()
            if (receive == 'sr'):
                self.transmit('sb')
                self.connect = True