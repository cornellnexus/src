import motor_controller
import time

# mc=motor_controller.MotorController()

# mc.setup()

# mc.turn_left()


# stopTime1 = time.time() + 3
# while time.time() < stopTime1:
	# mc.turn_right()

# stopTime2 = time.time() + 2
# while time.time() < stopTime2:
	# mc.stop()
	
# stopTime3 = time.time() + 3
# while time.time() < stopTime3:
	# mc.turn_left()
	
pid=motor_controller.PidGpio(5,15,15,5,5)

stopTime4 = time.time() + 5
while time.time() < stopTime4:
	pid.motors(0,30)
	
stopTime5 = time.time() + 5
while time.time() < stopTime5:
	pid.motors(0,0)
	
stopTime6 = time.time() + 5
while time.time() < stopTime6:
	pid.motors(20,30)
