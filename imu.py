import time
import board
import busio
import adafruit_lsm9ds1
from gpio import * 

class IMU: 
  def __init__(self, i2c):
    self.i2c = busio.I2C(imu_scl, imu_sda)
    self.imu = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
    self.acc = imu.acceleration
    self.mag = imu.magnetic 
    self.gyro = imu.gyro 

  #returns acc, mag, gyro data formatted in a dictionary
  def get_imu():
    combined_data = imu_format(self.acc, self.mag, self.gyro)
    return combined_data

  #helper function to combine IMU sensors together
  def imu_format(acc, mag, gyro):
      imu_dict = {
        "acc" : tuple(acc),
        "mag" : tuple(mag),
        "gyro" : tuple(gyro), 
      }
      return imu_dict


#Writing to CSV: 

# csv_data = [] #initialize empty list to store imu data
# original_time = time.time()

  # #helper function to print sensor data in x, y, z
  # def pretty_print(data):
  #     data = list(data)
  #     coords = {
  #       "x" : data[0], 
  #       "y" : data[1], 
  #       "z" : data[2]
  #     }
  #     return coords

# while time.time() < original_time + 15:
#     acc = imu.acceleration
#     mag = imu.magnetic
#     gyro = imu.gyro
      
#     combined_data = imu_format(acc, mag, gyro)
#     print(combined_data)
#     time.sleep(0.1)

#     csv_data.append(combined_data)

# with open('imu_360.csv', 'w') as imu_file:
#     for datum in csv_data: 
#         imu_file.write(str(datum) + '\n')
#         time.sleep(0.1)
