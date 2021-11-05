import serial
import time
import json
from imu import *
from gps import *
#---------------------------README-------------------------
# This file needs to go on the raspberry pi
#----------------------------------------------------------

# set up serial port communication
port = serial.Serial(port="/dev/ttyS0", baudrate=57600, timeout=1)
port.flush()
imu_sense = IMU()
gps_sense = GPS() # Need to make sure serial port doesn't conflict with IMU

#test_dict = {"one": 1, "two": 2, "three": 3}

# Send data from IMU if True. Else send data from GPS
imu_send = False

while True:
  if imu_send == True:
    combined_data = imu_sense.get_imu()	# get imu data in dict format
    json_object = str(combined_data) #json.dumps(combined_data) # serialize the data
    print(json_object)
    port.write((json_object + "\n").encode("utf-8"))
    time.sleep(0.05)
  else:
    combined_data = gps_sense.get_imu()
    json_object = str(combined_data) #json.dumps(combined_data) # serialize the data
    print(json_object)
    port.write((json_object + "\n").encode("utf-8"))
    time.sleep(1)