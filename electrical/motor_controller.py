import time
#from engine.robot import Robot
if False:  # change to True when running code on robot
    import RPi.GPIO as GPIO

class BasicMotorController:
    """ 
    BasicMotorController contains pinouts to configure motor controller, as well as 
    commands to physically move the robot. 
    """

    def __init__(self, is_sim):
        # raspberry pi motor driver pinouts
        self.in1 = 5
        self.in2 = 6
        self.in3 = 19
        self.in4 = 26
        self.enA = 13  # PWM pin
        self.enB = 12  # PWM pin
        self.is_sim = is_sim

    # checks all of the robot movements are functioning properly
    def setup(self):
        if not is_sim:
            GPIO.setmode(GPIO.BCM) #raspberry pi pinout reading mode
            GPIO.setup([self.in1, self.in2], GPIO.OUT, initial=GPIO.LOW)  # In1, In2, In3, In4
            GPIO.setup([self.enA, self.enB], GPIO.OUT)  # EnA, EnB
            # create object digital to analog conversion for PWM on port 25 at 1KHz
            self.e1 = GPIO.PWM(self.enA, 600)
            self.e2 = GPIO.PWM(self.enB, 600)
            self.e1.start(50)
            self.e2.start(50)      

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
            GPIO.output([self.in1], GPIO.HIGH)
            GPIO.output([self.in2], GPIO.LOW)

    # reverses the robot
    def reverse(self):
        if self.is_sim:
            print('reverse')
        else:
            GPIO.output([self.in2], GPIO.LOW)
            GPIO.output([self.in1], GPIO.LOW)

    # turns the robot left for 1 second
    def turn_left(self):
        if self.is_sim:
            print('turn_left')
        else:
            GPIO.output([self.in1], GPIO.LOW)
            GPIO.output([self.in2], GPIO.HIGH)
            self.e1.start(50)
            self.e2.start(100)


    # turns the robot right for 1 second
    def turn_right(self):
        if self.is_sim:
            print('turn_right')
        else:
            GPIO.output([self.in1], GPIO.HIGH)
            GPIO.output([self.in2], GPIO.LOW)
            self.e1.start(100)
            self.e2.start(50)


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
    def __init__(self, wheel_radius, vm_load1, vm_load2, L, R, is_sim):
        self.wheel_radius = wheel_radius
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
        self.is_sim

        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.in1, self.in2, self.in3, self.in4],GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup([self.enA, self.enB], GPIO.OUT)  # EnA, EnB

        self.p1 = GPIO.PWM(self.enA, 50)
        self.p2 = GPIO.PWM(self.enB, 50)

        # Start with 0% duty cycle
        self.p1.start(0)
        self.p2.start(0)

    # Initialize the robot's motors to 0 voltage. Used when powering the robot on.
    def setup(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.in1, self.in2, self.in3, self.in4],
                    GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup([self.enA, self.enB], GPIO.OUT)  # EnA, EnB

        self.p1 = GPIO.PWM(self.enA, 50)
        self.p2 = GPIO.PWM(self.enB, 50)

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
            dc1 = omega_right / self.vm_load1
            dc2 = omega_left / self.vm_load2
        except:
            raise ZeroDivisionError("vm_load1 or vm_load2 is zero")

        if dc1 > 100:
            dc1 = 100
        if dc2 > 100:
            dc2 = 100

        if not self.is_sim:
            self.p1.ChangeDutyCycle(dc1)
            self.p2.ChangeDutyCycle(dc2)
        else: 
            print("dc1: ", dc1, "and dc2: ", dc2)
        
if __name__=="__main__":
    mtr_ctrl = BasicMotorController()
    mtr_ctrl.setup()
    end_time = time.time() + 5
    while time.time() < end_time:
        mtr_ctrl.go_forward()

