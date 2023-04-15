import RPi.GPIO as GPIO
import time
from enum import Enum

class Breakbeam:
    ACCEPTABLE_TIME = 50 # time to wait to confirm a full break
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
            GPIO.setup(BEAM_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(BEAM_PIN, GPIO.BOTH, callback=self.break_beam_callback)
        self.beam_broken = False
        self.blocked_sensors = set()
    
    def break_beam_callback(self, channel):
        """
        Returns True if the beam at channel has been broken.
        Parameters: channel - the pin used by this sensor
        """
        self.beam_broken = not GPIO.input(channel)
        if self.beam_broken:
            self.timer(channel)
            self.check_half()
            self.check_full()
        # print("50%: " + self.check_half())
        # print("100%: " + self.check_full())
        return self.beam_broken
    
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
        while time.time() - start > Breakbeam.ACCEPTABLE_TIME:
            pass
        else:
            self.blocked_sensors.add(Breakbeam.BEAM_PINS(channel).name)
            # return channel (for testing)
        
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
message = input("Press enter to quit\n\n")
GPIO.cleanup()