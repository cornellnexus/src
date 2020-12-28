import RPi.GPIO as GPIO
import time

#Motor Driver
in1 = 27
in2 = 29
in3 = 31
in4 = 33
enA = 35
enB = 37

#IMU
imu_sda = 3
imu_scl = 5

#Sonar
sonar_trig = 16
sonar_echo = 18 #double check this order

#TODO:
#No Module GPIO
#pulseIn for sonar sensor
#Time for each turn (to makes sure robot turns ie. 90º exactly

def setup():
    GPIO.setmode(GPIO.BOARD)
    #Motor Driver
    GPIO.setup([in1, in2, in3, in4], GPIO.OUT, initial=GPIO.LOW) #In1, In2, In3, In4
    GPIO.setup([enA, enB], GPIO.OUT) #EnA, EnB

    e1 = GPIO.PWM(enA, 600)    # create object D2A for PWM on port 25 at 1KHz
    e2 = GPIO.PWM(enB, 600)

    e1.start(100)
    e2.start(100)

    #IMU
    GPIO.setup(imu_tx, GPIO.OUT)
    GPIO.setup(imu,rx, GPIO.IN)

    # RF 
    # GPS

    # Sonar
    # GPIO.setup(sonar_trig, GPIO.OUT)
    # GPIO.setup(sonar_echo, GPIO.IN)

# ----------------------------- MOTOR DRIVER ---------------------------------

def stopMotors():
    GPIO.output([in1, in2, in3, in4], GPIO.LOW) #In1, In2, In3, In4

def goForward():
    GPIO.output([in1, in4],GPIO.LOW)
    GPIO.output([in2, In3], GPIO.HIGH)

    """ Need to restructure code. Pwm objects go out of scope after function ends, so pwm
    will stop when function stops. Could make this file a class and have p1, p2 as member variables
    in the object so that their scope is global. Also, constructor would handle all the setup. 
    p1 = GPIO.PWM(enA, 1000) #1000Hz (Arduino Uno is 976)
    p2 = GPIO.PWM(enB, 1000) #1000Hz (Arduino Uno is 976)

    p1.start(78) #78% on. Equivalent to 200/255 in arduino
    p2.start(78) #78% on. Equivalent to 200/255 in arduino
    """

def reverse():
    GPIO.output([in2, In3], GPIO.LOW)
    GPIO.output([in1, in4],GPIO.HIGH)

    """
    p1 = GPIO.PWM(enA, 1000) #1000Hz (Arduino Uno is 976)
    p2 = GPIO.PWM(enB, 1000) #1000Hz (Arduino Uno is 976)

    p1.start(78) #78% on. Equivalent to 200/255 in arduino
    p2.start(78) #78% on. Equivalent to 200/255 in arduino
    """

def turnLeft():
    GPIO.output([in1, in2, in4],GPIO.LOW)
    GPIO.output([in3], GPIO.LOW)
    #analogWrite equivalent in rPi

    """
    p1 = GPIO.PWM(enA, 1000) #1000Hz (Arduino Uno is 976)
    p2 = GPIO.PWM(enB, 1000) #1000Hz (Arduino Uno is 976)

    p1.start(59) #59% on. Equivalent to 150/255 in arduino
    p2.start(98) #98% on. Equivalent to 250/255 in arduino
    """

def turnRight():
    GPIO.output([in1, in3, in4],GPIO.LOW)
    GPIO.output([in2], GPIO.LOW)
    #analogWrite equivalent in rPi

    """
    p1 = GPIO.PWM(enA, 1000) #1000Hz (Arduino Uno is 976)
    p2 = GPIO.PWM(enB, 1000) #1000Hz (Arduino Uno is 976)

    self.p1.start(98) #98% on. Equivalent to 250/255 in arduino
    self.p2.start(59) #59% on. Equivalent to 150/255 in arduino
    """

def stop():
    GPIO.output([in1, in2, in3, in4],GPIO.LOW)

