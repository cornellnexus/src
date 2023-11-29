import RPi.GPIO as GPIO
import time
from enum import Enum


def ackerman_calculations (steer_ang, left_ang, right_ang, outside_ang, inside_ang, track_width, wheel_base, steering_ration):
    pass

class Breakbeam:
    ACCEPTABLE_TIME = 3  # time to wait to confirm a full break

    # class BEAM_PINS(Enum):
    #     HALF1 = 17
    #     HALF2 = 22
    #     FULL1 = 23
    #     FULL2 = 24

    def __init__(self, pins):
        """
        Initializes each sensor to detect changes. Initializes beam_broken to False.
        Creates an empty list for the beams that are broken.
        Attribute blocked_sensors: set of sensors that have been fully blocked.
        """
        for BEAM_PIN in pins:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(BEAM_PIN.value, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(
                BEAM_PIN.value, GPIO.BOTH, callback=self.break_beam_callback
            )
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
        # print("50% full: " + str(self.check_half()))
        # print("100% full: " + str(self.check_full()))

    def timer(self, pins, channel):
        """
        Gets the current time and calculates the time that has passed, then compares
        this to the ACCEPTABLE_TIME to determine whether the broken beam was just
        tripped once or if it is fully broken. If fully broken, add it to the set
        blocked_sensors. If the sensor becomes unblocked during tis ACCEPTABLE_TIME,
        break out of the while-else loop.
        Parameters: channel - the pin used by this sensor
        """
        start = time.time()
        # loops until ACCEPTABLE_TIME and then goes into the else statement once
        while time.time() - start < Breakbeam.ACCEPTABLE_TIME:
            if GPIO.input(channel):
                break
        else:
            self.blocked_sensors.add(pins(channel).name)
            self.blocked_sensors_check()
            # print(self.blocked_sensors)

    def blocked_sensors_check(self, pins):
        """
        Checks whether the sensors in the set blocked_sensors are still blocked.
        Uses list TO_BE_REMOVED to store the sensors that have been unblocked and
        then remove them from the set of blocked sensors.
        """
        TO_BE_REMOVED = []  # sensors to remove due to being unbroken
        for sensor in self.blocked_sensors:
            if GPIO.input(pins[sensor].value):
                TO_BE_REMOVED.append(sensor)

        for sensor in TO_BE_REMOVED:
            self.blocked_sensors.remove(sensor)

    def check_half(self, pin_half_one, pin_half_two):
        """
        Returns whether the 50% sensors are in the set of fully blocked sensors
        """
        return (
            len(self.blocked_sensors) < 4
            and pin_half_one in self.blocked_sensors
            and pin_half_two in self.blocked_sensors
        )

    def check_full(self):
        """
        Returns whether the 100% sensors are in the set of fully blocked sensors.
        """
        return len(self.blocked_sensors) == 4


# Allows to exit program and clean up Pi for testing
# breakbeam = Breakbeam()
# message = input("Press enter to quit\n\n")
# GPIO.cleanup()
