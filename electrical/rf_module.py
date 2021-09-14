import serial 
import time 

""" This module receives the serially transmitted data from the rf module"""
class RadioSession:
    def __init__(self, port='/dev/ttyS0', device = 0):
        self.ser = serial.Serial('/dev/ttyS0', 57600)
        self.baudrate = 57600
        self.port = port
        self.connect = False
        self.device_list = [0, 1, 2] #to do: figure out better way to do this
        self.device = device

    def receive_data(self):
        byte_data = self.ser.readline()
        return byte_data

    def transmit_data(self,t_data):
        cast_data = bytes((str(t_data) + '\n'), encoding= 'utf-8')
        self.ser.write(cast_data)

    def startup(self): 
        pass
        """
        device 0 sends data to device 1
        device 1 reads the data from device 0
        device 1 confirms data received by sending back to 0
        device 0 confirms receives data from device 1
        device 0 sends required data to device 1
        Idea for handshakes:
        state machine with states:
        initialization- establish radio connection through three way handshake, checksums/
            rate we send data
        the rest- start constand transmition/recive, checksums
        """