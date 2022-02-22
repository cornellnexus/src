from electrical.motor_controller import MotorController, MotorPID
import time

#TODO: Add documentation here

class MotorControllerTest:	
	def __init__(self): 
		self.mc = MotorController()
		
	def motor_controller_test_one(self): 
		stopTime1 = time.time() + 3
		while time.time() < stopTime1:
			self.mc.turn_right()
	
	def motor_controller_test_two(self): 
		stopTime2 = time.time() + 2
		while time.time() < stopTime2:
			self.mc.stop()
	
	def motor_controller_test_two(self): 
		stopTime3 = time.time() + 3
		while time.time() < stopTime3:
			self.mc.turn_left()

class MotorPIDTest: 
	def __init__(self): 
		self.pid = MotorPID.PidGpio(5,15,15,5,5)
	
	def motor_pid_test_one(self): 
		stopTime4 = time.time() + 5
		while time.time() < stopTime4:
			self.pid.motors(0,30)
	
	def motor_controller_test_two(self): 
		stopTime5 = time.time() + 5
		while time.time() < stopTime5:
			self.pid.motors(0,0)
	
	def motor_controller_test_two(self): 
		stopTime6 = time.time() + 5
		while time.time() < stopTime6:
			self.pid.motors(20,30)
