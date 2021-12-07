import time
import board
import busio
import adafruit_lsm9ds1

#---------------------------README-------------------------
# This file needs to go on the raspberry pi
#----------------------------------------------------------

class IMU:
    
    i2c = busio.I2C(board.SCL, board.SDA)
    imu = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

    def __init__(self):
         self.acc = 0
         self.mag = 0
         self.gyro = 0

    """get_imu returns acc, mag, gyro data formatted in a dictionary"""
    
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

    """imu_format is a helper function to combine IMU sensors together"""

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


# Simple starter test program that just prints IMU values in a neat fashion
'''
sensor = IMU()

#helper function to print sensor data in x, y, z
def pretty_print(data):
    data = list(data)
    coords = {
      "x" : data[0],
      "y" : data[1],
      "z" : data[2]
    }
    return coords

while True:

    combined_data = sensor.get_imu()
    
    format_data_acc = pretty_print(combined_data["acc"])
    format_data_mag = pretty_print(combined_data["mag"])
    format_data_gyr = pretty_print(combined_data["gyro"])
    
    print(format_data_acc["z"])
#    print(format_data_acc["y"])
#    print(format_data_acc["x"])

#    print(format_data_mag["z"])
#    print(format_data_mag["y"])
#    print(format_data_mag["x"])

#    print(format_data_gyr["z"])
#    print(format_data_gyr["y"])
#    print(format_data_gyr["x"])
    print("\n")
    time.sleep(0.05)
'''