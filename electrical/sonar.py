import digitalio
from adafruit_mcp230xx.mcp23008 import MCP23008
import board
import busio
import time

"""
Authors: 
  Luis Martinez, George Maidhof, Nicholas Papapanou

Description: 
  This class is meant for use with the JSN-SR04T ultrasonic sensor in conjunction
  with the MCP23008 port expander. The module allows for a maximum of 4 US sensors
  to simultaneously communicate with the RPI through the port expander.

How to use:
  For each added ultrasonic sensor, you need an instance of this class. The input
  into the constructor specifies which sensor you are referring to, and abstracts
  away any information about what pins each sensor is connected to on the port
  expander. For example,

  sensor1 = Ultrasonic(0)

  defines that we are using a new ultrasonic sensor and is connected to the 0th
  slot on the port expander. By "slot" I mean which pair of pins on the port expander
  the sensor is connected to

Wiring Description (Should not change):
  The following information should only need to be referenced if you want to know
  which sensor you are referring to or if the circuit becomes disassembled.

  Ultrasonic Sensor 0 --> TRIG to GP0, ECHO to GP1
  Ultrasonic Sensor 1 --> TRIG to GP2, ECHO to GP3
  Ultrasonic Sensor 2 --> TRIG to GP4, ECHO to GP5
  Ultrasonic Sensor 3 --> TRIG to GP6, ECHO to GP7

  MCP23008 SDA   --> RPI SDA
  MCP23008 SCL   --> RPI SCL
  MCP23008 A0    --> GND
  MCP23008 A1    --> GND
  MCP23008 A2    --> GND
  MCP23008 RESET --> 3.3V

                             top of MCP23008
                          _________\/__________
                          |                   |
                      SCL -                   - 3.3V
                          |                   |
                      SDA -                   - GP7 (Ultrasonic sensor 3 ECHO)
                          |                   |
                       A2 -                   - GP6 (Ultrasonic sensor 3 TRIG)
                          |                   |
                       A1 -                   - GP5 (Ultrasonic sensor 2 ECHO)
                          |                   |
                       A0 -                   - GP4 (Ultrasonic sensor 2 TRIG)
                          |                   |
                    RESET -                   - GP3 (Ultrasonic sensor 1 ECHO)
                          |                   |
                       NC -                   - GP2 (Ultrasonic sensor 1 TRIG)
                          |                   |
                     INT  -                   - GP1 (Ultrasonic sensor 0 ECHO)
                          |                   |
                      GND -                   - GP0 (Ultrasonic sensor 0 TRIG)
                          |___________________|
  
"""

class Ultrasonic:
    
    i2c = busio.I2C(board.SCL, board.SDA)
    mcp = MCP23008(i2c)
    
    def __init__(self, sensor):
        self.sensor = sensor
        
        if (self.sensor == 0):
          self.trigger = self.mcp.get_pin(0)
          self.echo= self.mcp.get_pin(1)
        if (self.sensor == 1):
          self.trigger = self.mcp.get_pin(2)
          self.echo= self.mcp.get_pin(3)
        if (self.sensor == 2):
          self.trigger = self.mcp.get_pin(4)
          self.echo= self.mcp.get_pin(5)
        if (self.sensor == 3):
          self.trigger = self.mcp.get_pin(6)
          self.echo= self.mcp.get_pin(7)
        self.echo.direction = digitalio.Direction.INPUT
        self.trigger.direction = digitalio.Direction.OUTPUT
        
    def distance(self):
        # set Trigger to HIGH
        self.trigger.value = True
     
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        self.trigger.value = False
     
        StartTime = time.time()
        StopTime = time.time()
     
        # save StartTime
        while self.echo.value == False:
            StartTime = time.time()
        # save time of arrival
        while self.echo.value == True:
            StopTime = time.time()
     
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
        return distance
