"""This module serially reads the data from the IMU sensors""" 

import time
import board
import busio
import adafruit_lsm9ds1
from gpio import * 

i2c = busio.I2C(imu_scl, imu_sda)
imu = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

accelerometer = list(imu.acceleration)
magnetometer = list(imu.magnetic)
gyroscope = list(imu.gyro)
temp = imu.temperature

csv_data = []
collect_time = time.time() + 300


#[pretty_print(list)] is used to pretty print a [list] with three componenents 
# (x, y, z) for coordinate labelling
def pretty_print(list):
    coords = {
      "x" : list[0], 
      "y" : list[1], 
      "z" : list[2]
    }
    return coords

#[imu_print()] takes type unit and prints a dictionary of combined imu data 
def imu_format():
    imu_dict = {
      "acc" : pretty_print(accelerometer),
      "mag" : pretty_print(magnetometer),
      "gyro" : pretty_print(gyroscope), 
      "temp" : temp
    }
    print(imu_dict)
    return imu_dict

# [write_to_csv(data_lst) writes to "imu_noise.csv" imu data elements in 
# data_lst

def write_to_csv(data_lst): 
  while time.time() < collect_time: 
    data_lst.append(imu_format())
  with open('imu_noise.csv', 'w') as imu_file:
    for data in data_lst: 
      imu_file.write(str(data) + '\n')
            
# write_to_csv(csv_data)

while True:
    write_to_csv(csv_data)
    time.sleep()

