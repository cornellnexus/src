""" This module imports serial data from sonar sensor """

# TODO: figure out the Pulse_in function originally in Arduino code. 
from gpio import * 

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