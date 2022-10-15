# Odometry is for calculating linear and angular displacement of robot

import RPi.GPIO as GPIO
import math

test = True
counter_right = 0
counter_left = 0
ppr=500 #pulse per revolution of motor
r=5     #dummy variable for radius of wheels
L=15    #dummy variable for distance from one wheel to center of robot

def pulse_handler_right(pin):
	if (counter_right==10):
		odometry(counter_right, counter_left, r, L)     #Do odometry when right wheel gets 10 pulses
    else:
        counter_right+=1
       

def pulse_handler_left(pin):
    counter_left+=1     

def odometry(counter_right, counter_left, r, L):
	dtheta_right=(2*math.pi*counter_right)/ppr  #angular displacement of left wheel
	dtheta_left=(2*math.pi*counter_left)/ppr    #angular displacement of right wheel
	ds_right=dtheta_right*r                          #change in linear displacement of left wheel
	ds_left=dtheta_left*r                            #change in linear displacement of right wheel
	linear_dis=(ds_right+ds_left)/2                  #linear displacement of robot
	angular_dis=(ds_right-ds_left)/(2*L)             #angular displacement of robot
	if (test):
		print("linear displacement is"+str(linear_dis)+"\n"+"angular displacement is"+str(angular_dis))
    counter_right=0
    counter_left=0
        

if __name__ == "__main__":
	#global freq
	GPIO.setmode(GPIO.BCM) 
	GPIO.setup(26, GPIO.IN) #encoder pi pinout for back right motor
	GPIO.setup(9,GPIO.IN) #encoder pi pinout for back left motor
	try:
    #freq: countable events per rev = 3415,92
    #freq: cycles per rev = countable events per rev / 4 = 853.92, where 4 indicates the edges for square wave
		GPIO.add_event_detect(26, GPIO.RISING, callback=pulse_handler_right)  
		GPIO.add_event_detect(9, GPIO.RISING, callback=pulse_handler_left)  
    except:
        print("check code and setup, something is wrong")
    finally:
        GPIO.cleanup()
