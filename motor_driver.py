import RPi.GPIO as GPIO
import time

#Motor Driver
in1 = 29
in2 = 31
in3 = 35
in4 = 37
enA = 33
enB = 32

#IMU
imu_tx = 14
imu_rx  = 15

#Sonar
sonar_trig = 23
sonar_echo = 24

#TODO:
#pulseIn for sonar sensor
#Time for each turn (to makes sure robot turns ie. 90º exactly
#Double Check the tx, rx for the IMU (is IMU UART?)

GPIO.setmode(GPIO.BOARD)
#Motor Driver
GPIO.setup([in1, in2, in3, in4], GPIO.OUT, initial=GPIO.LOW) #In1, In2, In3, In4
GPIO.setup([enA, enB], GPIO.OUT) #EnA, EnB

e1 = GPIO.PWM(enA, 500)    # create object D2A for PWM on port 25 at 1KHz
e2 = GPIO.PWM(enB, 500)

e1.start(60)
e2.start(60)

#IMU
GPIO.setup(imu_tx, GPIO.OUT)
GPIO.setup(imu,rx, GPIO.IN)

# Telemetry
# GPS
# Sonar

def stopMotors():
    GPIO.output([in1, in2, in3, in4], GPIO.LOW) #In1, In2, In3, In4

def goForward():
    GPIO.output([in1, in4],GPIO.LOW)
    GPIO.output([in2, In3], GPIO.HIGH)

def reverse():
    GPIO.output([in2, In3], GPIO.LOW)
    GPIO.output([in1, in4],GPIO.HIGH)

def turnLeft():
    GPIO.output([in1, in2, in4],GPIO.LOW)
    GPIO.output([in3], GPIO.LOW)
    #analogWrite equivalent in rPi

def turnRight():
    GPIO.output([in1, in3, in4],GPIO.LOW)
    GPIO.output([in2], GPIO.LOW)
    #analogWrite equivalent in rPi

def stop():
    GPIO.output([in1, in2, in3, in4],GPIO.LOW)
