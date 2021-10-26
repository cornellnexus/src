import serial
import time
import json
from imu import *

#---------------------------README-------------------------
# This file needs to go on the raspberry pi
#----------------------------------------------------------

# set up serial port communication
port = serial.Serial(port="/dev/ttyS0", baudrate=57600, timeout=1)
port.flush()
imu_sense = IMU()

#test_dict = {"one": 1, "two": 2, "three": 3}

while True:
	combined_data = imu_sense.get_imu()	# get imu data in dict format
	json_object = str(combined_data) #json.dumps(combined_data) # serialize the data
	print(json_object)
	port.write((json_object + "\n").encode("utf-8"))
	time.sleep(0.05)