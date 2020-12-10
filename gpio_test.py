import RPi.GPIO as GPIO
import time

class robotControl:

    #Motor Driver. Variables shared by all instances of class
    in1 = 27
    in2 = 29
    in3 = 31
    in4 = 33
    enA = 35
    enB = 37

    #IMU. Variables shared by all instances of class
    imu_tx = 14
    imu_rx  = 15

    #Sonar. Variables shared by all instances of class
    sonar_trig = 23
    sonar_echo = 24

    def __init__(self, p1, p2):
        GPIO.setmode(GPIO.BOARD)
        #Motor Driver 
        GPIO.setup([in1, in2, in3, in4], GPIO.OUT, initial=GPIO.LOW) #In1, In2, In3, In4
        GPIO.setup([enA, enB], GPIO.OUT) #EnA, EnB

        #IMU
        GPIO.setup(imu_tx, GPIO.OUT)
        GPIO.setup(imu,rx, GPIO.IN)
        # Telemetry
        # GPS
        # Sonar
        GPIO.setup(sonar_trig, GPIO.OUT)
        GPIO.setup(sonar_echo, GPIO.IN)

        self.p1 = GPIO.PWM(enA, 1000) #1000Hz (Arduino Uno is 976)
        self.p2 = GPIO.PWM(enB, 1000) #1000Hz (Arduino Uno is 976)
        self.p1.start(0) #starts with duty cycle of 0 (0% on)
        self.p1.start(0) #starts with duty cycle of 0 (0% on)

    def stopMotors(self):
        GPIO.output([in1, in2, in3, in4], GPIO.LOW) #In1, In2, In3, In4

    def goForward(self):
        GPIO.output([in1, in4],GPIO.LOW)
        GPIO.output([in2, In3], GPIO.HIGH)

        """ Need to restructure code. Pwm objects go out of scope after function ends, so pwm
        will stop when function stops. Could make this file a class and have p1, p2 as member variables
        in the object so that their scope is global. Also, constructor would handle all the setup. 
        """
        
        self.p1.ChangeDutyCycle(78) #78% on. Equivalent to 200/255 in arduino
        self.p2.ChangeDutyCycle(78) #78% on. Equivalent to 200/255 in arduino

    def reverse(self):
        GPIO.output([in2, In3], GPIO.LOW)
        GPIO.output([in1, in4],GPIO.HIGH)

        self.p1.ChangeDutyCycle(78) #78% on. Equivalent to 200/255 in arduino
        self.p2.ChangeDutyCycle(78) #78% on. Equivalent to 200/255 in arduino

    def turnLeft(self):
        GPIO.output([in1, in2, in4],GPIO.LOW)
        GPIO.output([in3], GPIO.LOW)
        #analogWrite equivalent in rPi

        self.p1.ChangeDutyCycle(59) #59% on. Equivalent to 150/255 in arduino
        self.p2.ChangeDutyCycle(98) #98% on. Equivalent to 250/255 in arduino

    def turnRight(self):
        GPIO.output([in1, in3, in4],GPIO.LOW)
        GPIO.output([in2], GPIO.LOW)
        #analogWrite equivalent in rPi

        self.p1.ChangeDutyCycle(98) #98% on. Equivalent to 250/255 in arduino
        self.p2.ChangeDutyCycle(59) #59% on. Equivalent to 150/255 in arduino

    def stop(self):
        GPIO.output([in1, in2, in3, in4],GPIO.LOW)

    # ----------------------------- SONAR SENSOR ---------------------------------
    """
    sleep_time = 2*(10**-6) #2 microseconds
    hold_sound_time = 10*(10**-6)

    while true:
        GPIO.output(sonar_trig, GPIO.LOW) #make sure trig pin set to low
        time.sleep(sleep_time)
        GPIO.output(sonar_trig, GPIO.HIGH) #trigger sends burst
        time.sleep(hold_sound_time)
        GPIO.output(sonar_trig, GPIO.LOW)

        duration = 2 #figure out pulseIn in rPi
        #duration = pulseIn(echoPin, HIGH); //times sound wave length
        distance = (duration * 0.0343)/2; #0.0343 = speed of sound
        # print(distance)
        sleep(0.001)
    """