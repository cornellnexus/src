import time
import board
import busio
import adafruit_lsm9ds1
from gpio import * 

i2c = busio.I2C(imu_scl, imu_sda)
imu = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

csv_data = [] #initialize empty list to store imu data
original_time = time.time()

def pretty_print(data):
    data = list(data)
    coords = {
      "x" : data[0], 
      "y" : data[1], 
      "z" : data[2]
    }
    return coords

def imu_format(acc, mag, gyro):
    imu_dict = {
      "acc" : pretty_print(acc),
      "mag" : pretty_print(mag),
      "gyro" : pretty_print(gyro), 
    }
    return imu_dict

while time.time() < original_time + 15:
    acc = imu.acceleration
    mag = imu.magnetic
    gyro = imu.gyro
      
    combined_data = imu_format(acc, mag, gyro)
    print(combined_data)
    time.sleep(0.1)

    csv_data.append(combined_data)

with open('imu_360.csv', 'w') as imu_file:
    for datum in csv_data: 
        imu_file.write(str(datum) + '\n')
        time.sleep(0.1)
