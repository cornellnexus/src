from sys import is_finalizing
import serial 
import time 


class RadioSession:
    """
    RadioSession establishes the function calls needed between two devices.

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
    #setup function called on the robot
    def setup_robot(self): 
        while (not self.connected): #while rpi not connected to base station
            self.transmit_data('setup', 'sr')
            receive = self.receive_data()
            # print("connected: ", self.connected, "received: ",  receive)
            if (receive == 'sb'):
                # print("finished!")
                self.connected = True

    #setup function called on the base station 
    #TODO: call this in the GUI class
    def setup_basestation(self):
        # while (not self.connected): #while rpi not connected to base station
        #     data = self.ser.read()
        #     # print(data)
        #     if (data == 'sr'):
        #         self.transmit_data('setup','sb') 
        #         self.connected = True
        print("Hello!")


    