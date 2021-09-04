""" This controller module for detecting if a robot is moving straight
    using encoder data from motors
"""

from gpiozero import Robot, DigitalInputDevice
from time import sleep

SAMPLETIME = 1
TARGET = 45
KP = 0.02
KD = 0.01
KI = 0.005


class Encoder(object):
    def __init__(self, pin):
        self._value = 0
        encoder = DigitalInputDevice(pin)
        encoder.when_activated = self._increment
        encoder.when_deactivated = self._increment

    def reset(self):
        self._value = 0

    def _increment(self):
        self._value += 1

    @property
    def value(self):
        return self._value


r = Robot((10, 9), (8, 7))
e1 = Encoder(17)
e2 = Encoder(18)

m1_speed = 1.0
m2_speed = 1.0
r.value = (m1_speed, m2_speed)

e1_prev_error = 0
e2_prev_error = 0
e1_sum_error = 0
e2_sum_error = 0

while True:
    e1_error = TARGET - e1.value
    e2_error = TARGET - e2.value

    # Potential Fix: the integral branch doesn't seem to be taking integral, it's just summing up the errors.
    # I think the example code simplifies it to sum of error instead of actually taking integral
    # e1_integral = e1_integral_prior + e1_error * SAMPLETIME
    # e2_integral = e2_integral_prior + e2_error * SAMPLETIME

    e1_derivative = (e1_error - e1_prev_error) / SAMPLETIME
    e2_derivative = (e2_error - e2_prev_error) / SAMPLETIME

    m1_speed += (e1_error * KP) + e1_derivative * KD + (e1_sum_error * KI)
    m2_speed += (e2_error * KP) + e2_derivative * KD + (e2_sum_error * KI)

    m1_speed = max(min(1, m1_speed), 0)
    m2_speed = max(min(1, m2_speed), 0)

    r.value = (m1_speed, m2_speed)

    print("e1 {} e2 {}".format(e1.value, e2.value))
    print("m1 {} m2 {}".format(m1_speed, m2_speed))

    e1.reset()
    e2.reset()
    sleep(SAMPLETIME)

    e1_prev_error = e1_error
    e2_prev_error = e2_error
    e1_sum_error += e1_error
    e2_sum_error += e2_error
