import RPi.GPIO as GPIO
import time
from enum import Enum


class Breakbeam:
    ACCEPTABLE_TIME = 3 # time to wait to confirm a full break
    class BEAM_PINS(Enum):
        HALF1 = 17
        HALF2 = 22
        FULL1 = 23
        FULL2 = 24

    def __init__(self):
        """
        Initializes each sensor to detect changes. Initializes beam_broken to False.
        Creates an empty list for the beams that are broken.
        """
        for BEAM_PIN in Breakbeam.BEAM_PINS:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(BEAM_PIN.value, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(BEAM_PIN.value, GPIO.BOTH, callback=self.break_beam_callback)
        self.blocked_sensors = set()
    
    def break_beam_callback(self, channel):
        """
        Returns True if the beam at channel has been broken.
        Parameters: channel - the pin used by this sensor
        """
        if not GPIO.input(channel):
            self.timer(channel)
            self.check_half()
            self.check_full()
        # print(self.check_half())
        # print(self.check_full())
    
    def timer(self, channel):
        """
        Returns the pin number of this sensor.
        Gets the current time and calculates the time that has passed, then compares
        this to the ACCEPTABLE_TIME to determine whether the broken beam was just
        tripped once or if it is fully broken. If fully broken, add it to the set
        blocked_sensors.
        Parameters: channel - the pin used by this sensor
        """
        start = time.time()
        # loops until ACCEPTABLE_TIME and then goes into the else statement once
        while time.time() - start < Breakbeam.ACCEPTABLE_TIME:
            if(GPIO.input(channel)):
                break
        else:
            self.blocked_sensors.add(Breakbeam.BEAM_PINS(channel).name)
            self.blocked_sensors_check()
            print(self.blocked_sensors)
        
    def blocked_sensors_check(self):
        TO_BE_REMOVED = []
        for sensor in self.blocked_sensors:
            if(GPIO.input(Breakbeam.BEAM_PINS[sensor].value)):
                TO_BE_REMOVED.append(sensor)
                
        for sensor in TO_BE_REMOVED:
            self.blocked_sensors.remove(sensor) 
        
        
    def check_half(self):
        """
        Returns whether the 50% sensors are in the set of fully blocked sensors
        """
        if len(self.blocked_sensors) < 4 and Breakbeam.BEAM_PINS(17).name in self.blocked_sensors and Breakbeam.BEAM_PINS(22).name in self.blocked_sensors:
            # bucket has been 50% filled
            return True
        else:
            return False
        
    def check_full(self):
        """
        Returns whether the 100% sensors are in the set of fully blocked sensors.
        """
        if len(self.blocked_sensors) == 4:
            # bucket has been 100% filled
            return True
        else:
            return False
        

# Allows to exit program and clean up Pi for testing
breakbeam = Breakbeam()
message = input("Press enter to quit\n\n")
GPIO.cleanup()
