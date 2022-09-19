from sys import is_finalizing
import serial 
import time 


class RadioModule:
    """
    RadioModule establishes the function calls needed between two devices.

    Communication package headings used when sending messages or parsing received messages:
    """
    def __init__(self, serial):
        self.connected = False 
        self.ser = serial

    #receive data reads the serially transmitted data the other radio device is sending
    def receive_data(self):
        #serial is reading by line; imself.serant that data must be sent with a newline appended to it '/n' 
        byte_data = self.ser.readline() 
        return byte_data

    def transmit_data(self,data, packet_type):
        assert isinstance(packet_type, str)
        data = packet_type + str(data) + ' \n' 
        cast_data = bytes(data, encoding= 'utf-8')
        self.ser.write(cast_data)

    #implementation of 2-way handshake between the robot and basestation/gui for serial data transmission
    #setup function called on the robot
    """
    Send message two-ways with one way being with setup_basestation and the other being with setup_robot. 
    Print Success both ways if message is received from both basestation and robot. 

    """
    def setup_robot(self): 
        self.ser.flush()
        self.ser.flushInput()
        data =  'handshake_robot'       
        cast_data = bytes(data,encoding = 'utf-8')
        self.ser.write(cast_data) #write data to basestation
        t_end = time.time() + 5
        line = None
        while time.time() < t_end: #trying to read the line in 5 seconds
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8')
                break
        #if line read from robot is "handshake_computer", we write "Success" to the robot and read the line sent from robot
        if (line == "handshake_computer"): 
            success = "Success"
            cast_success = bytes(success, encoding = "utf-8")
            self.ser.write(cast_success)
            t_end = time.time() + 5
            line = None
            while time.time() < t_end:
                if self.ser.in_waiting > 0:
                        line = self.ser.readline().decode('utf-8')
                        break
            #if line read from robot is Success, we print "Success". Else, we print "Failure."
            if (line == "Success"):
                    print("Success")
                    self.connected = True
            else:
                    print("Failure") 
                    self.connected = False 
        #if line read from robot is not "handshake_computer", we print "Failure"
        else:
            print("Failure")
            self.connected = False

    #setup function called on the base station 
    #TODO: call this in the GUI class
    """
    Send message two-ways with one way being with setup_basestation and the other being with setup_robot. 
    Print Success both ways if message is received from both basestation and robot. 
    """
    def setup_basestation(self):
        self.ser.flush()
        self.ser.flushInput()
        t_end = time.time() + 5
        line = None
        while time.time() < t_end:  #trying to read the line in 5 seconds
            if self.ser.in_waiting > 0: 
                line = self.ser.readline().decode("utf-8")
                break
        message = "handshake_computer"
        cast_message = bytes(message, encoding = "utf-8")
        self.ser.write(cast_message) #write message to robot
        #if line read from robot is "handshake_robot", we read the next line
        if (line == "handshake_robot"): 
            t_end = time.time() + 5
            line = None
            while time.time() < t_end:
                if self.ser.in_waiting > 0: 
                    line = self.ser.readline().decode("utf-8")
                    break
            #if the next line read is "Success" we print Success. Else, we print "Failure"
            if(line == "Success"):
                print("Success")
                self.connected = True
            else:
                print("Failure")
                self.connected = False
            #we write "Success" to the robot 
            success = "Success"
            cast_success = bytes(success, encoding = "utf-8")
            self.ser.write(cast_success)
        #if line read from robot is not "handshake robot" we print "Failure"
        else:
            print("Failure")
            self.connected = False


    