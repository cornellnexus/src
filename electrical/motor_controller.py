import time
from engine.robot import Robot
if True: #change to True when running code on robot
    import RPi.GPIO as GPIO


class MotorController:
    """ 
    MotorController contains pinouts to configure motor controller, as well as 
    commands to physically move the robot. 
    """
    def __init__(self, robot):
        #raspberry pi motor driver pinouts
        self.in1 = 5
        self.in2 = 6
        self.in3 = 19
        self.in4 = 26
        self.enA = 13   #PWM pin
        self.enB = 12   #PWM pin
        self.is_sim = robot.is_sim

    # checks all of the robot movements are functioning properly
    def setup(self):
        if self.is_sim: 
            self.go_forward()
            self.turn_left()
            self.turn_right()
            self.reverse()
            self.stop()
        else: 
            GPIO.setmode(GPIO.BCM) #raspberry pi pinout reading mode
            GPIO.setup([self.in1, self.in2, self.in3, self.in4], GPIO.OUT, initial=GPIO.LOW)  # In1, In2, In3, In4
            GPIO.setup([self.enA, self.enB], GPIO.OUT)  # EnA, EnB
            self.e1 = GPIO.PWM(self.enA, 600)  # create object digital to analog conversion for PWM on port 25 at 1KHz
            self.e2 = GPIO.PWM(self.enB, 600)
            self.e1.start(100)
            self.e2.start(100)            

    # stops the robot 
    def stop(self):
        if self.is_sim: 
            print('stop')
        else:
            self.e1.stop()
            self.e2.stop()
        
    # moves the robot forward
    def go_forward(self):
        if self.is_sim: 
            print('go_forward')
        else: 
            GPIO.output([self.in1, self.in4], GPIO.HIGH)
            GPIO.output([self.in2, self.in3], GPIO.HIGH)

    # reverses the robot
    def reverse(self):
        if self.is_sim: 
            print('reverse')
        else: 
            GPIO.output([self.in2, self.in3], GPIO.LOW)
            GPIO.output([self.in1, self.in4], GPIO.LOW)

    # turns the robot left for 1 second
    def turn_left(self):
        if self.is_sim: 
            print('turn_left')
        else: 
            GPIO.output([self.in2, self.in4], GPIO.LOW)
            GPIO.output([self.in1, self.in3], GPIO.HIGH)
        time.sleep(1)

    # turns the robot right for 1 second
    def turn_right(self):
        if self.is_sim:
            print('turn_right')
        else: 
            GPIO.output([self.in1, self.in3], GPIO.LOW)
            GPIO.output([self.in2, self.in4], GPIO.HIGH)
        time.sleep(1)


class MotorPID:
    """ 
    MotorPID contains pinouts to configure motor controller, PID tuning values for the 
    motors, and can set motor torque according to input angular and linear velocities.

    Attributes: 
        robot: robot object 
        wheel_r: the wheel radius 
        vm_load1: maximum velocity can drive load1 #TODO: make this more descriptive
        vm_load2: maximum velocity can drive load2 #TODO: make this more descriptive
        L: radius of left motor #TODO: double check this 
        R: radius of right motor #TODO: double check this
    """
    def __init__(self, robot, wheel_r, vm_load1, vm_load2, L, R):
        self.in1 = 5
        self.in2 = 6
        self.enA = 13   #PWM
        self.enB = 12   #PWM
        self.is_sim = robot.is_sim
        #super().__init__(robot)
        self.wheel_r = wheel_r
        self.vm_load1 = vm_load1
        self.vm_load2 = vm_load2
        self.L = L
        self.R = R
        self.in1 = 5
        self.in2 = 6
        self.in3 = 19
        self.in4 = 26
        self.enA = 13
        self.enB = 12


    # Change duty cycle for motors based on angular and linear velocities
    def motors(self, omega, vel):
        if omega == 0:
            vr = vel
            vl = vel
        else:
            vr = omega * (self.R + self.L / 2)
            vl = omega * (self.R - self.L / 2)

        omega_r = vr * self.wheel_r
        omega_l = vl * self.wheel_r

        # Define and cap duty cycles if they are above max
        dc1 = omega_r / self.vm_load1
        dc2 = omega_l / self.vm_load2
        if dc1 > 100:
            dc1 = 100
        if dc2 > 100:
            dc2 = 100
        
        if self.is_sim:
            self.p1.ChangeDutyCycle(dc1)
            self.p2.ChangeDutyCycle(dc2)
        else: 
            print("dc1: ", dc1, "and dc2: ", dc2)

    def setup(self):
        if self.is_sim: 
            self.motors(0, 0)
        else:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup([self.in1, self.in2, self.in3, self.in4], GPIO.OUT, initial=GPIO.LOW)  
            GPIO.setup([self.enA, self.enB], GPIO.OUT)  # EnA, EnB

            self.p1 = GPIO.PWM(self.enA, 50)
            self.p2 = GPIO.PWM(self.enB, 50)

            # Initialize PWM duty cycles as 0
            self.p1.start(0)
            self.p2.start(0)

            self.motors(0, 0)
