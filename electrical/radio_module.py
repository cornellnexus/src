from sys import is_finalizing
import serial 
import time 


class RadioModule:
    """
    RadioModule establishes the function calls needed between two devices.

    Communication package headings used when sending messages or parsing received messages:
    'sr': start (robot)
    'sb': start (base station)
    'dr': data (robot) 
    'db': data (base station) 
    """
    def __init__(self, serial):
        self.connected = False 
        self.ser = serial

    #receive data reads the serially transmitted data the other radio device is sending
    def receive_data(self):
        #serial is reading by line; important that data must be sent with a newline appended to it '/n' 
        byte_data = self.ser.readline() 
        return byte_data

    def transmit_data(self,data, packet_type):
        assert isinstance(packet_type, str)
        data = packet_type + str(data) + ' \n' 
        cast_data = bytes(data, encoding= 'utf-8')
        self.ser.write(cast_data)

    #implementation of 2-way handshake between the robot and basestation/gui for serial data transmission
    def setup_robot(self): 
        while (not self.connected): #while rpi not connected to base station
            self.transmit_data('setup', 'sr')
            receive = self.receive_data()
            if (receive == 'sb'):
                self.connected = True

    def setup_basestation(self):
        if self.is_sim:
            print("Hello!")
        else:
            while (not self.connected): #while rpi not connected to base station
                data = self.ser.read()
                if (data == 'sr'):
                    self.transmit_data('setup','sb') 
                    self.connected = True
        


    