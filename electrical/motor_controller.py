from engine.robot import Robot
import time
if False: #change to True when running code on robot
    import RPi.GPIO as GPIO

""" MotorController contains pinouts to configure motor controller, as well as 
    commands to physically move the robot. """
class MotorController:
    def __init__(self, robot):
        #raspberry pi motor driver pinouts
        self.in1 = 5
        self.in2 = 6
        self.in3 = 19
        self.in4 = 26
        self.enA = 13
        self.enB = 12
        self.is_sim = robot.is_sim

    # checks all of the robot movements are functioning properly
    def setup(self):
        if not self.is_sim: 
            GPIO.setmode(GPIO.BCM) #raspberry pi pinout reading mode
            GPIO.setup([self.in1, self.in2, self.in3, self.in4], GPIO.OUT, initial=GPIO.LOW)  # In1, In2, In3, In4
            GPIO.setup([self.enA, self.enB], GPIO.OUT)  # EnA, EnB
            e1 = GPIO.PWM(self.enA, 600)  # create object digital to analog conversion for PWM on port 25 at 1KHz
            e2 = GPIO.PWM(self.enB, 600)
            e1.start(100)
            e2.start(100)
        if self.is_sim: 
            self.go_forward()
            self.turn_left()
            self.turn_right()
            self.reverse()
            self.stop()

    # stops the robot 
    def stop(self):
        if self.is_sim: 
            print('stop')
        else:
            GPIO.output([self.in1, self.in2, self.in3, self.in4],GPIO.LOW)
        
    # moves the robot forward
    def go_forward(self):
        if self.is_sim: 
            print('go_forward')
        else: 
            GPIO.output([self.in1, self.in4],GPIO.LOW)
            GPIO.output([self.in2, self.in3], GPIO.HIGH)

    # reverses the robot
    def reverse(self):
        if self.is_sim: 
            print('reverse')
        else: 
            GPIO.output([self.in2, self.in3], GPIO.LOW)
            GPIO.output([self.in1, self.in4],GPIO.HIGH)

    # turns the robot left for 1 second
    def turn_left(self):
        if self.is_sim: 
            print('turn_left')
        else: 
            GPIO.output([self.in2, self.in4],GPIO.LOW)
            GPIO.output([self.in1, self.in3], GPIO.HIGH)
        time.sleep(1)

    # turns the robot right for 1 second
    def turn_right(self):
        if self.is_sim:
            print('turn_right')
        else: 
            GPIO.output([self.in1, self.in3],GPIO.LOW)
            GPIO.output([self.in2, self.in4], GPIO.HIGH)
        time.sleep(1)

# testing the functions to run under 10 seconds 
# stopTime = time.time() + 5
# while time.time() < stopTime:
#     go_forward()
# # # 
# # stop()
# # 

# # cleans up all the ports used for motor driver 
# GPIO.cleanup()
