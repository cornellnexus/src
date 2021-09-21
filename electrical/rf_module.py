from sys import is_finalizing
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
        self.SYN = False 
        self.SYNACK = False 
        self.ACK = False 
        self.FIN = False

    def receive_data(self):
        byte_data = self.ser.readline()
        return byte_data

    def transmit_data(self,t_data):
        cast_data = bytes((str(t_data) + '\n'), encoding= 'utf-8')
        self.ser.write(cast_data)

    #implementation of 3-way handshake
    #raspberry pi serves as "client" in 3-way handshake
    #returns True if Rpi successfully set up 
    def startup_rpi(self): 
        if (self.device != 0): #self.device should be 0
            return "error, set device to 0" 
        while (not self.connect): #while rpi not connected to base station
            if (not self.SYNACK): #receiving device has not confirmed data received
                self.transmit_data("SYN") #send first data
                receive = self.receive_data() #receive
                if (receive == "SYNACK"):
                    self.SYNACK = True 
            else: 
                if (not self.FIN):
                    self.transmit_data("ACK")
                    receive = self.receive_data() #receive
                    if (receive == "FIN"):
                        self.FIN = True
                        self.connect = True
        return True
            
    def startup_base(self):
        if (self.device != 1): 
            return "error, set device to 1"
        while (not self.connect): #while rpi not connected to base station
            receive = self.receive_data()
            if (not self.SYN): #not receiving first SYN command
                if (receive == "SYN"):
                    self.transmit("SYNACK")
                    self.SYN = True
            elif (not self.ACK):
                if (receive == "ACK"):
                    self.transmit("FIN")
                    self.ACK = True 
        return True 
