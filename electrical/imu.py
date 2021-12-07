import time
import board
import busio
import adafruit_lsm9ds1

""" Module that includes functions for IMU sensor"""
class IMU:
    
    i2c = busio.I2C(board.SCL, board.SDA)
    imu = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

    def __init__(self):
         self.acc = 0
         self.mag = 0
         self.gyro = 0

    """get_imu: returns acc, mag, gyro data formatted in a dictionary"""
    
    def set_num_dec(self, num, reading):
        x = round(reading[0], num)
        y = round(reading[1], num)
        z = round(reading[2], num)
        return x, y, z

    def get_imu(self):
        self.acc = self.set_num_dec(3, tuple(self.imu.acceleration))
        self.gyro = self.set_num_dec(3, tuple(self.imu.gyro))
        self.mag = self.set_num_dec(3, tuple(self.imu.magnetic))
        combined_data = self.imu_format(self.acc, self.mag, self.gyro)
        return combined_data

    """imu_format: a helper function to combine IMU sensors together"""

    def imu_format(self, acc, mag, gyro):
        imu_dict = {
         "acc": acc,
         "mag": mag,
         "gyro": gyro
        }
        return imu_dict
    
    def write_to_csv(data_arr, file):
        with open(file, "w") as imu_file:
            for datum in data_arr: 
                imu_file.write(str(datum) + '\n')

    
    """ startup: function that checks the imu data for acc, mag, gyro   
        returns True when IMU is setup properly, False if not"""
    def startup(self):
        pass #need to define what starting up an IMU entails 
        #calibrate function?