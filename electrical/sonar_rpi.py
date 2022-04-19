#Libraries
import RPi.GPIO as GPIO
import time

"""
Authors: 
  Jerry Jin, Luis Martinez

Description: 
  This class is meant for use with the JSN-SR04T ultrasonic sensor connected
  directly to the RPI, mainly for testing purposes.

How to use:
  For each added ultrasonic sensor, you need an instance of this class. The input
  into the constructor specifies which sensor you are referring to, and abstracts
  away any information about what pins each sensor is connected to on the port
  expander. For example,

  sensor1 = Ultrasonic(<echo pin>, <trigger pin>)

  defines that we are using a new ultrasonic sensor and its echo pin is connected
  to <echo pin> on the raspberry pi, and the trigger pin in connected to <trigger pin>
  on the raspberry pi. Since we are operating in BCM, <echo pin> and <trigger pin>
  refer to GPIO numbers not pin numbers.

Wiring Description:
  For each ultrasonic sensor, connect the ECHO and TRIGGER pins to their own GPIO
  pin on the RPI. The 5V pin should go to 5V, and the GND pin to GND  
"""

class UltrasonicRPI:

  def __init__(self, echo, trig):
    self.echo = echo
    self.trig = trig

    #GPIO Mode (BCM)
    GPIO.setmode(GPIO.BCM)
  
    #set GPIO direction (IN / OUT)
    GPIO.setup(self.trig, GPIO.OUT)
    GPIO.setup(self.echo, GPIO.IN)
  
  # Returns the distance from and object in cm (range is 20-600)
  def distance(self):
      # set Trigger to HIGH
      GPIO.output(self.trig, True)
  
      # set Trigger after 0.01ms to LOW
      time.sleep(0.00001)
      GPIO.output(self.trig, False)
  
      StartTime = time.time()
      StopTime = time.time()
  
      # save StartTime
      while GPIO.input(self.echo) == 0:
          StartTime = time.time()
  
      # save time of arrival
      while GPIO.input(self.echo) == 1:
          StopTime = time.time()
  
      # time difference between start and arrival
      TimeElapsed = StopTime - StartTime
      # multiply with the sonic speed (34300 cm/s)
      # and divide by 2, because there and back
      distance = (TimeElapsed * 34300) / 2
  
      return distance