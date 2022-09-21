import RPi.GPIO as GPIO
import time
 
"""
Tests one ultrasonic sensor. 
Run this test script, place an object (ie. hand) slightly away from the ultrasonic sensor. 
Observe the printed distance values from the test script to see if they are the expected distance
the object is away from the ultrasonic sensor.
"""


#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER = 1
GPIO_ECHO = 7
GPIO_TRIGGER2 = 8
GPIO_ECHO2 = 25

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_TRIGGER2, GPIO.OUT)
GPIO.setup(GPIO_ECHO2, GPIO.IN)

def distance(trigger, echo):
    # set Trigger to HIGH
    GPIO.output(trigger, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(trigger, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(echo) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(echo) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance
 
if __name__ == '__main__':
    try:
        while True:
            dist = distance(GPIO_TRIGGER, GPIO_ECHO)
            dist2 = distance(GPIO_TRIGGER2, GPIO_ECHO2)
            print ("Measured Distance for GPIO is = %.1f cm" % dist)
            print ("Measured Distance for GPIO2 is = %.1f cm" % dist2)
            time.sleep(0.1)
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup() 