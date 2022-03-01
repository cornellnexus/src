from electrical.motor_controller import MotorController, MotorPID
import time

class MotorControllerTest:	
	"""
	MotorController Test checks that the motors are able to rotate in an expected direction.
	
	For this test to run properly, the wired connections of the GPIO pins must be set-up properly.

	Expected Outcome: Motor rotates one direction for three seconds, stops, rotates the opposite 
	direction for the next three seconds.
	"""
	def __init__(self): 
		self.mc = MotorController()
		
	def motor_controller_test_one(self): 
		stopTime = time.time() + 3
		while time.time() < stopTime:
			self.mc.turn_right()
	
	def motor_controller_test_two(self): 
		stopTime = time.time() + 2
		while time.time() < stopTime:
			self.mc.stop()
	
	def motor_controller_test_three(self): 
		stopTime = time.time() + 3
		while time.time() < stopTime:
			self.mc.turn_left()

	def run(self): 
		self.motor_controller_test_one()
		self.motor_controller_test_two()
		self.motor_controller_test_three()


class MotorPIDTest: 
	"""
	MotorPIDTest checks that the motors are tunable according to specific PID constants, as well as 
	are able to rotate for a respective omega and velocity value set. 

	For this test to run properly, the wired connections of the GPIO pins must be set-up properly.

	Expected Outcome: Motor rotates for 5 seconds at omega = 0 and velocity = 30, stops, then proceeds
	to rotate at omega = 20 and velocity = 30. 
	"""
	def __init__(self): 
		self.pid = MotorPID.PidGpio(5,15,15,5,5)
	
	def motor_pid_test_one(self): 
		stopTime = time.time() + 5
		while time.time() < stopTime:
			self.pid.motors(0,30)
	
	def motor_pid_test_two(self): 
		stopTime = time.time() + 5
		while time.time() < stopTime:
			self.pid.motors(0,0)
	
	def motor_pid_test_three(self): 
		stopTime = time.time() + 5
		while time.time() < stopTime:
			self.pid.motors(20,30)

	def run(self): 
		self.motor_pid_test_one()
		self.motor_pid_test_two()
		self.motor_pid_test_three()
