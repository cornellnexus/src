from electrical.motor_controller import PidGpio
from engine.sensor_module import SensorModule 
import time
import RPi.GPIO as GPIO

motor_controller = PidGpio()
sensor_module = SensorModule(False)

stopTime = time.time() + 10 

while time.time() < stopTime:
    #Move at a constant arbitrary velocity (hard-coded to 5)
    motor_controller.spin_motors(0, 5)
    sensor_module.update_gps_data()
    sensor_module.update_imu_data()

motor_controller.spin_motors(0,0)

GPIO.cleanup()
