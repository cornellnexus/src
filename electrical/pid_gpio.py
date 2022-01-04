# ===============================
# AUTHOR:        Luis Martinez
# CREATE DATE:   12/30/2021
# PURPOSE:       Code to convert linear and angular velocities into PWM signals for motors. Part of PID loop. This code
#                interacts directly with the pins on the raspberry pi, so connections to the motor/motor controller
#                should be checked with the pin definitions in gpio.py
# SPECIAL NOTES: Meant to be run on raspberry pi.
# ===============================

import gpio
import RPi.GPIO as GPIO

class PID_GPIO:

    # Define wheel radius and
    def __init__(self, wheel_r, vm_load1, vm_load2, L, R):
        self.wheel_r   = wheel_r
        self.vm_load1  = vm_load1
        self.vm_load2  = vm_load2
        self.L = L
        self.R = R

        GPIO.setup([gpio.in1, gpio.in2, gpio.in3, gpio.in4], GPIO.OUT, initial=GPIO.LOW)  # In1, In2, In3, In4
        GPIO.setup([gpio.enA, gpio.enB], GPIO.OUT)  # EnA, EnB

        self.p1 = GPIO.PWM(gpio.enA, 50)
        self.p2 = GPIO.PWM(gpio.enB, 50)

        self.p1.ChangeDutyCycle(0)  # where 0.0 <= dc <= 100.0
        self.p2.ChangeDutyCycle(0)  # where 0.0 <= dc <= 100.0

        self.p1.start()
        self.p2.start()

    # Change duty cycle for motors based on angular and linear velocities
    def motors(self, omega, vel):
        if omega == 0:
            vr = vel
            vl = vel
        else:
            vr = omega*(self.R + self.L/2)
            vl = omega*(self.R - self.L/2)

        omega_r = vr*self.wheel_r
        omega_l = vl*self.wheel_r

        # Define and cap duty cycles if they are above max
        dc1 = omega_r/self.vm_load1
        dc2 = omega_l/self.vm_load2
        if dc1 > 100:
            dc1 = 100
        if dc2 > 100:
            dc2 = 100

        self.p1.ChangeDutyCycle(dc1)
        self.p2.ChangeDutyCycle(dc2)


