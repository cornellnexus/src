from electrical.motor_controller import MotorController, MotorPID
import time

#TODO: Add documentation here

mc = MotorController()

mc.setup()

#test case 1 (write a def for this or make into testcase)

stopTime1 = time.time() + 3
while time.time() < stopTime1:
	mc.turn_right()

stopTime2 = time.time() + 2
while time.time() < stopTime2:
	mc.stop()
	
stopTime3 = time.time() + 3
while time.time() < stopTime3:
	mc.turn_left()
	
pid = MotorPID.PidGpio(5,15,15,5,5)

stopTime4 = time.time() + 5
while time.time() < stopTime4:
	pid.motors(0,30)
	
stopTime5 = time.time() + 5
while time.time() < stopTime5:
	pid.motors(0,0)
	
stopTime6 = time.time() + 5
while time.time() < stopTime6:
	pid.motors(20,30)