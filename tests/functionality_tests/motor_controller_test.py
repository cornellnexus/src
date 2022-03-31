from electrical.motor_controller import BasicMotorController, MotorController  
import time

class BasicMotorControllerTest:	
	"""
	BasicMotorController Test checks that the motors are able to rotate in an expected direction.
	
	For this test to run properly, the wired connections of the GPIO pins must be set-up properly.

	Expected Outcome: Motor rotates one direction for three seconds, stops, rotates the opposite 
	direction for the next three seconds.
	"""
	def __init__(self): 
		self.mc = BasicMotorController()
		
	def motor_controller_test_one(self): 
		stop_time = time.time() + 3
		while time.time() <	stop_time:
			self.mc.turn_right()
	
	def motor_controller_test_two(self): 
		stop_time = time.time() + 2
		while time.time() <	stop_time:
			self.mc.stop()
	
	def motor_controller_test_three(self): 
		stop_time = time.time() + 3
		while time.time() <	stop_time:
			self.mc.turn_left()

	def run(self): 
		self.motor_controller_test_one()
		self.motor_controller_test_two()
		self.motor_controller_test_three()


class MotorControllerTest: 
	"""
	MotorControllerTest checks that the motors are able to rotate for a respective omega and velocity value set. 

	For this test to run properly, the wired connections of the GPIO pins must be set-up properly.

	Expected Outcome: Motor rotates for 5 seconds at omega = 0 and velocity = 30, stops, then proceeds
	to rotate at omega = 20 and velocity = 30. 
	"""
	def __init__(self): 
		self.motor_controller = MotorController(None, 5,15,15,5,5)
	
	def test_one(self): 
		stop_time = time.time() + 5
		while time.time() <	stop_time:
			self.motor_controller.motors(0,30)
	
	def test_two(self): 
		stop_time = time.time() + 5
		while time.time() <	stop_time:
			self.motor_controller.motors(0,0)
	
	def test_three(self): 
		stop_time = time.time() + 5
		while time.time() <	stop_time:
			self.motor_controller.motors(20,30)

	def run(self): 
		self.test_one()
		self.test_two()
		self.test_three()
