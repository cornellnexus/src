import RPi.GPIO as GPIO
import math


class Odometry:
    """
    This class is used for calculating linear and angular displacement of robot's
    motors by getting encoder data
    """

    def __init__(self):
        self.counter_right = 0
        self.counter_left = 0
        self.ppr = 500  # motor pulse per revolution
        self.wheel_radius = 5  # variable for radius of wheels
        self.wheel_robot_dist = (
            15  # variable for distance from one wheel to center of robot
        )

    def setup(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(26, GPIO.IN)  # encoder pi pinout for back right motor
        GPIO.setup(9, GPIO.IN)  # encoder pi pinout for back left motor

    def odometry(self):
        # reset counters
        self.counter_right = 0
        self.counter_left = 0
        # change in angular displacement of left back wheel and right back wheel
        dtheta_right = (2 * math.pi * self.counter_right) / self.ppr
        dtheta_left = (2 * math.pi * self.counter_left) / self.ppr
        # change in linear displacement of left back wheel and right back wheel
        ds_right = dtheta_right * self.wheel_radius
        ds_left = dtheta_left * self.wheel_radius
        # average linear displacement and angular displacement of robot
        # linear_dis=(ds_right+ds_left)/2
        # angular_dis=(ds_right-ds_left)/(2*wheel_robot_dist)
        return dtheta_right, dtheta_left, ds_right, ds_left

    def pulse_handler_right(self):
        if self.counter_right == 10:
            self.odometry()

    def pulse_handler_left(self):
        self.counter_left += 1

    def get_encoder_data(self):
        try:
            # freq: countable events per rev = 3415,92
            # freq: cycles per rev = countable events per rev / 4 = 853.92, where 4 indicates the edges for square wave
            GPIO.add_event_detect(26, GPIO.RISING, callback=self.pulse_handler_right)
            GPIO.add_event_detect(9, GPIO.RISING, callback=self.pulse_handler_left)
        except:
            print("check code and setup, something is wrong")
        # finally:
        #     GPIO.cleanup()
