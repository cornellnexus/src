from motor_controller import BasicMotorController
import time
import math
import RPi.GPIO as GPIO
test = True
ppr=854 #pulse per revolution of motor
r=5     #dummy variable for radius of wheels
L=15    #dummy variable for distance from one wheel to center of robot
pin = 26
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin,GPIO.IN)
    
class BasicMotorControllerTest:

    def __init__(self): 
        self.mc = BasicMotorController(False, 1)
        self.mc.setup('very slow') # CHANGE SPEED HERE ('very slow','slow','medium','fast','max')
        self.counter_right = 0

    # Turns right for 3 seconds
    def motor_controller_turn_right(self): 
        stop_time = time.time() + 3
        while time.time() <	stop_time:
            self.mc.turn_right()

    def motor_controller_test_stop(self): 
        stop_time = time.time() + 2
        while time.time() <	stop_time:
            self.mc.stop()
    
    # Turns left for 3 seconds
    def motor_controller_turn_left(self): 
        stop_time = time.time() + 3
        while time.time() <	stop_time:
            self.mc.turn_left()

    # Moves forward for 3 seconds
    def motor_controller_forward(self):
        GPIO.add_event_detect(pin,GPIO.RISING)
        stop_time = time.time() + 3 # CHANGE DURATION HERE
        while time.time() <	stop_time:
            self.mc.go_forward()
            if GPIO.event_detected(pin):
                self.counter_right+=1
        print("count is "+str(self.counter_right))
        GPIO.cleanup()

    def run(self):
        self.motor_controller_forward()
        
    def count_pulse(self,pin):
#     global counter_right
        self.counter_right+=1


# class MotorControllerTest: 
# 	"""
# 	MotorControllerTest checks that the motors are able to rotate for a respective omega and velocity value set. 
# 
# 	For this test to run properly, the wired connections of the GPIO pins must be set-up properly.
# 
# 	Expected Outcome: Motor rotates for 5 seconds at omega = 0 and velocity = 30, stops, then proceeds
# 	to rotate at omega = 20 and velocity = 30. 
# 	"""
# 	def __init__(self): 
# 		self.motor_controller = MotorController(None, 5,15,15,5,5)
# 	
# 	def test_straight(self): 
# 		stop_time = time.time() + 5
# 		while time.time() <	stop_time:
# 			self.motor_controller.spin_motors(0,30) pin numbering mode using GPIO.setmode(GPIO.BOARD) or GP
# 	
# 	def test_stop(self): 
# 		stop_time = time.time() + 5
# 		while time.time() <	stop_time:
# 			self.motor_controller.spin_motors(0,0)
# 	
# 	def test_turning_and_moving(self): 
# 		stop_time = time.time() + 5
# 		while time.time() <	stop_time:
# 			self.motor_controller.spin_motors(20,30)
# 
# 	def run(self): 
# 		self.test_straight()
# 		self.test_stop()
# 		self.test_turning_and_moving()
# 
# #BasicMotorControllerTest.init()


 #encoder pi pinout for back right motor
	#GPIO.setup(9,GPIO.IN) #encoder pi pinout for back left motor
# global counter_right
# counter_right = 0

def pulse_handler_right(pin):
    global counter_right
    if (counter_right==10):
        odometry( r, L)     #Do odometry when right wheel gets 10 pulses
        print("check3")
    else:     
        counter_right+=1
        print("check2")
       

#def pulse_handler_left(pin):
#    counter_left+=1     

# def odometry(r, L):
    # global counter_right
    # dtheta_right=(2*math.pi*counter_right)/ppr  #angular displacement of right wheel
    # print("check4")

    # print("angle is"+str(dtheta_right))		#+"\n"+"angular displacement is"+str(angular_dis)
    # print("check5")
    # counter_right=0
    #counter_left=0
    
test= BasicMotorControllerTest()    
test.run()


# if __name__ == "__main__":
#     GPIO.setmode(GPIO.BCM) 
#     GPIO.setup(pin,GPIO.IN)	
#     print("check0")
#     try:
# 
#             #freq: countable events per rev = 3415,92
#             #freq: cycles per rev = countable events per rev / 4 = 853.92, where 4 indicates the edges for square wave
#         GPIO.add_event_detect(pin,GPIO.RISING,callback=pulse_handler_right)  
#             #GPIO.add_event_detect(9, GPIO.RISING, callback=pulse_handler_left)
#         print("check1")
# 
#     except:
#         print("check code and setup, something is wrong")
#     finally:
#         GPIO.cleanup()
