import RPi.GPIO as GPIO

in1 = 27
in2 = 29
in3 = 31
in4 = 33
enA = 35
enB = 37

imu_tx = 14
imu_rx  = 15

#TODO:
#Time for each turn (to makes sure robot turns ie. 90º exactly
#PWM frequency for how fast wheels are moving
#Double Check the tx, rx for the IMU (is IMU UART?)

def setup():
    GPIO.setmode(GPIO.BOARD)
    #L298N
    GPIO.setup([in1, in2, in3, in4], GPIO.OUT, initial=GPIO.LOW) #In1, In2, In3, In4
    GPIO.setup([enA, enB], GPIO.OUT) #EnA, EnB
    GPIO.PWM(enA, 1000)    # create object D2A for PWM on port 25 at 1KHz
    GPIO.PWM(enB, 1000)
    #IMU
    GPIO.setup(imu_tx, GPIO.OUT)
    GPIO.setup(imu,rx, GPIO.IN)
    # Telemetry
    # GPS

# ----------------------------- MOTOR DRIVER ---------------------------------

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
