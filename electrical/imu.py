import time
import board
import busio
import adafruit_lsm9ds1

""" Module that includes functions for IMU sensor"""
class IMU:
    def __init__(self, init_i2c):
        i2c = init_i2c
        self.imu = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
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

    
    """ setup: function that checks the imu data for acc, mag, gyro   
        returns True when IMU is setup properly, False if not"""
    def setup(self):
        imu_data = []
        count = 0
        while (len(imu_data) < 25): 
            count += 1 
            data = self.get_imu()
            if (data.get("acc") != 0 and data.get("mag") != 0 and data.get("gyro")!=0): 
                imu_data.append(data)
            if (count > 250): 
                return False
        return True 
