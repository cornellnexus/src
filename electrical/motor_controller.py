import time
import math
#from engine.robot import Robot
# from engine.is_raspberrypi import is_raspberrypi
# if is_raspberrypi():
import RPi.GPIO as GPIO

class BasicMotorController:
    """ 
    BasicMotorController contains pinouts to configure motor controller, as well as 
    commands to physically move the robot. 
    """

    def __init__(self, is_sim, gear_ratio):
        # raspberry pi motor driver pinouts
        self.gpio_br = 5 # GPIO5, pin29, back right motor, DIR1
        self.gpio_bl = 6 # GPIO6, pin31, back left motor, DIR2
        self.gpio_fr = 23 # GPIO23, pin 16, front right motor, DIR1
        self.gpio_fl = 24 # GPIO24, pin 18, front right motor, DIR2
        self.pwm_pin_br = 13  # GPIO13(PWM1), pin 33, back right motor, PWM1
        self.pwm_pin_bl = 12  # GPIO12(PWM0), pin 32, back left motor, PWM2
        self.pwm_pin_fr = 25 # GPIO25, pin 22, front right motor, PWM1
        self.pwm_pin_fl = 8 # GPIO8, pin 24, front left motor, PWM2
        self.is_sim = is_sim # Change to False when running on robot, same with importing RPi.GPIO
        self.gear_ratio = gear_ratio
        self.front_radius = 0.15    # unit is meters
        self.back_radius = 0.21    # unit is meters

    # checks all of the robot movements are functioning properly
    def setup(self, speed):
        if not self.is_sim:
            GPIO.setmode(GPIO.BCM) #raspberry pi pinout reading mode
            # back motors
            GPIO.setup([self.gpio_br, self.gpio_bl], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup([self.pwm_pin_br, self.pwm_pin_bl], GPIO.OUT)
            # front motors
            GPIO.setup([self.gpio_fr, self.gpio_fl], GPIO.OUT, initial=GPIO.LOW)  # gpio_br, gpio_bl, gpio_fr, gpio_fl
            GPIO.setup([self.pwm_pin_fr, self.pwm_pin_fl], GPIO.OUT)  # pwm_pin_br, pwm_pin_bl
            # create object digital to analog conversion for PWM on port 25 at 1KHz
            self.back_right = GPIO.PWM(self.pwm_pin_br, 600)
            self.back_left = GPIO.PWM(self.pwm_pin_bl, 600)
            self.front_right = GPIO.PWM(self.pwm_pin_fr, 600)
            self.front_left = GPIO.PWM(self.pwm_pin_fl, 600)
            if speed == "very slow": #0.3 m/s
                self.back_right.start(10.85)
                self.back_left.start(10.85)
                self.front_right.start(15.89)
                self.front_left.start(15.89)
            elif speed == "slow": #0.6 m/s
                self.back_right.start(23.56)
                self.back_left.start(23.56)
                self.front_right.start(33.64)
                self.front_left.start(33.64)
            elif speed == "medium": #1 m/s
                self.back_right.start(40.51)
                self.back_left.start(40.51)
                self.front_right.start(57.31)
                self.front_left.start(57.31)
            elif speed == "fast": # 1.4 m/s
                self.back_right.start(57.46)
                self.back_left.start(57.46)
                self.front_right.start(80.98)
                self.front_left.start(80.98)
            elif speed == "max": # 1.7 m/s
                self.back_right.start(70.17)
                self.back_left.start(70.17)
                self.front_right.start(98.73)
                self.front_left.start(98.73)

    # stops the robot
    def stop(self):
        if self.is_sim:
            print('stop')
        else:
            self.back_right.stop()
            self.back_left.stop()
            self.front_right.stop()
            self.front_left.stop()

    # moves the robot forward
    def go_forward(self):
        if self.is_sim:
            print('go_forward')
        else:
            GPIO.output([self.gpio_br], GPIO.HIGH)
            GPIO.output([self.gpio_bl], GPIO.LOW)
            GPIO.output([self.gpio_fr], GPIO.HIGH)
            GPIO.output([self.gpio_fl], GPIO.LOW)

    # reverses the robot
    def reverse(self):
        if self.is_sim:
            print('reverse')
        else:
            GPIO.output([self.gpio_br], GPIO.LOW)
            GPIO.output([self.gpio_bl], GPIO.HIGH)
            GPIO.output([self.gpio_fr], GPIO.LOW)
            GPIO.output([self.gpio_fl], GPIO.HIGH)

    # turns the robot left for 1 second
    def turn_left(self):
        if self.is_sim:
            print('turn_left')
        else:
            GPIO.output([self.gpio_br], GPIO.LOW)
            GPIO.output([self.gpio_bl], GPIO.HIGH)
            self.back_right.start(50)
            self.back_left.start(100)


    # turns the robot right for 1 second
    def turn_right(self):
        if self.is_sim:
            print('turn_right')
        else:
            GPIO.output([self.gpio_br], GPIO.HIGH)
            GPIO.output([self.gpio_bl], GPIO.LOW)
            self.back_right.start(100)
            self.back_left.start(50)

    # converts duty cycle into revolution per second base on experiment
    # see motor pulse per revolution sheet for derivation (electrical folder)
    # call individually on each wheel
    def rps_given_dc(self, dc):
        motor_rps = (0.0537/3)*dc + 0.1/3
        wheel_rps = (1/self.gear_ratio)*motor_rps
        if (self.is_sim == True) :
            print("wheel revolution per second for duty cycle "+ dc + "is: " + wheel_rps)
        return wheel_rps
    
    # For an elapsed amount of time(seconds), wheel revolution per 
    # second, and wheel size(small, big) returns expected distance the wheel travels(meters)
    # call individually on each wheel (wheels at same side should return same value)
    def revolutions_to_distance(self, time, wheel_rps, wheel_size):
        if (wheel_size == "big"):
            radius = self.back_radius
        elif (wheel_size == "small"):
            radius = self.front_radius
        circumference = 2*math.pi*radius
        distance = circumference*wheel_rps*time
        if (self.is_sim == True) :
            print("Distance traveled in " + time + "seconds: " + distance + " meters")
        return distance


# TODO: figure out how to vary direction of wheels (gpio_br-gpio_fl) base on what we have
# now all gpio_br-gpio_fl is at LOW
class MotorController:
    """ 
    MotorController contains pinouts to configure motor controller and can set motor torque 
    according to input angular and linear velocities.
    Attributes: 
        robot: robot object 
        wheel_radius: the wheel radius 
        vm_load1: maximum velocity can drive load1 #TODO: make this more descriptive
        vm_load2: maximum velocity can drive load2 #TODO: make this more descriptive
        L: radius of left motor #TODO: double check this 
        R: radius of right motor #TODO: double check this
    """
    def __init__(self, vm_load1, vm_load2, L, R, is_sim, gear_ratio):
        self.vm_load1 = vm_load1
        self.vm_load2 = vm_load2
        self.L = L
        self.R = R
        # raspberry pi motor driver pinouts
        self.gpio_br = 5 # GPIO5, pin29, back right motor, DIR1
        self.gpio_bl = 6 # GPIO6, pin31, back left motor, DIR2
        self.gpio_fr = 23 # GPIO23, pin 16, front right motor, DIR1
        self.gpio_fl = 24 # GPIO24, pin 18, front right motor, DIR2
        self.pwm_pin_br = 13  # GPIO13(PWM1), pin 33, back right motor, PWM1
        self.pwm_pin_bl = 12  # GPIO12(PWM0), pin 32, back left motor, PWM2
        self.pwm_pin_fr = 25 # GPIO25, pin 22, front right motor, PWM1
        self.pwm_pin_fl = 8 # GPIO8, pin 24, front left motor, PWM2
        self.is_sim = is_sim    # Change to False when running on robot, same with importing RPi.GPIO
        self.gear_ratio = gear_ratio
        self.front_radius = 0.15    # unit is meters
        self.back_radius = 0.21    # unit is meters
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.gpio_br, self.gpio_bl, self.gpio_fr, self.gpio_fl],GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup([self.pwm_pin_br, self.pwm_pin_bl], GPIO.OUT)  # pwm_pin_br, pwm_pin_bl

        self.pwm_br = GPIO.PWM(self.pwm_pin_br, 50)
        self.pwm_bl = GPIO.PWM(self.pwm_pin_bl, 50)

        # Start with 0% duty cycle
        self.pwm_br.start(0)
        self.pwm_bl.start(0)

    # Initialize the robot's motors to 0 voltage. Used when powering the robot on.
    def setup(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.gpio_br, self.gpio_bl, self.gpio_fr, self.gpio_fl],
                    GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup([self.pwm_pin_br, self.pwm_pin_bl], GPIO.OUT)  # pwm_pin_br, pwm_pin_bl

        self.pwm_br = GPIO.PWM(self.pwm_pin_br, 25)
        self.pwm_bl = GPIO.PWM(self.pwm_pin_bl, 50)

    # converts the robot's overall calculated angular velocity and linear velocity
    # into the angular velocities for left and right sides of the robot
    def left_right_angular_vel(self, omega, vel):
        if omega == 0:
            vr = vel
            vl = vel
        else:
            vr = omega * (self.R + self.L / 2)
            vl = omega * (self.R - self.L / 2)

        omega_r = vr * self.wheel_radius
        omega_l = vl * self.wheel_radius
        return omega_r, omega_l

    # Change duty cycle (voltage applied) for motors based on
    # overall robot's angular and linear velocities

    def spin_motors(self, omega, vel):
        omega_right, omega_left = self.left_right_angular_vel(omega, vel)

        # Define and cap duty cycles if they are above max
        try:
            dc1 = omega_right / self.vm_load1 # duty cycle = angular velocity of wheel we need : maximum angular velocity we can achieve
            dc2 = omega_left / self.vm_load2
        except:
            raise ZeroDivisionError("vm_load1 or vm_load2 is zero")

        if dc1 > 100:
            dc1 = 100
        if dc2 > 100:
            dc2 = 100

        if not self.is_sim:
            self.pwm_br.ChangeDutyCycle(dc1)
            self.pwm_bl.ChangeDutyCycle(dc2)
        else:
            print('dc1: ', dc1, ' and dc2: ', dc2)
 
    # converts duty cycle into revolution per second base on experiment
    # see motor pulse per revolution sheet for derivation (electrical folder)
    # call individually on each wheel
    def rps_given_dc(self, dc):
        motor_rps = (0.0537/3)*dc + 0.1/3
        wheel_rps = (1/self.gear_ratio)*motor_rps
        if (self.is_sim == True) :
            print('Wheel revolutions per second for duty cycle '+ dc + ' is: ' + wheel_rps)
        return wheel_rps
    
    # For an elapsed amount of time(seconds), wheel revolution per 
    # second, and wheel size(small, big) returns expected distance the wheel travels(meters)
    # call individually on each wheel (wheels at same side should return same value)
    def revolutions_to_distance(self, time, wheel_rps, wheel_size):
        if (wheel_size == 'big'):
            radius = self.back_radius
        elif (wheel_size == 'small'):
            radius = self.front_radius
        circumference = 2*math.pi*radius
        distance = circumference*wheel_rps*time
        if (self.is_sim == True) :
            print('Distance traveled in ' + time + 'seconds: ' + distance + ' meters')
        return distance


