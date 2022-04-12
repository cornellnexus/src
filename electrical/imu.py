import time
if False: 
    import board
    import busio
    import adafruit_lsm9ds1

""" Module that includes functions for IMU sensor"""
class IMU:
    def __init__(self, init_i2c, is_sim):
        self.is_sim = is_sim
        if not self.is_sim: 
            i2c = init_i2c
            self.imu = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
            self.acc = 0
            self.mag = 0
            self.gyro = 0


    def set_num_dec(self, num, reading):
        """
        Returns rounded values of the [reading] to [num] digits.
        
        Parameters: [reading] is the data value from the IMU
                    [num] is the number of digits to round [reading] to.
        
        Preconditions: [reading] is an integer
                       [num] is an integer
    """
        x = round(reading[0], num)
        y = round(reading[1], num)
        z = round(reading[2], num)
        return x, y, z


    def get_imu(self):
        """
        Returns acc, mag, gyro data formatted in a dictionary.
        """
        if not self.is_sim: 
            self.acc = self.set_num_dec(3, tuple(self.imu.acceleration))
            self.gyro = self.set_num_dec(3, tuple(self.imu.gyro))
            self.mag = self.set_num_dec(3, tuple(self.imu.magnetic))
            combined_data = self.imu_format(self.acc, self.mag, self.gyro)
            return combined_data


    def imu_format(self, acc, mag, gyro):
        """
        Returns the IMU sensor readings of the accelerometer, magnetometer, and gyroscope
        as a dictionary for formatting purposes.

        Parameters: [acc] is the accelerometer X, Y, Z axis values as a 3-tuple of m/s^2 values
                    [mag] is the magnetometer X, Y, Z axis values as a 3-tuple of gauss values
                    [gyro] is the gyroscope X, Y, Z axis values as a 3-tuple of rad/s values
        
        Preconditions: 
                    [acc] is a tuple of three ints
                    [mag] is a tuple of three ints 
                    [gyro] is a tuple of three ints
        """
        imu_dict = {
         "acc": acc,
         "mag": mag,
         "gyro": gyro
        }
        return imu_dict
    

    def write_to_csv(data_arr, file):
        """
        Writes [data_arr] to a csv [file].
        """
        with open(file, "w") as imu_file:
            for datum in data_arr: 
                imu_file.write(str(datum) + '\n')

    
    def setup(self):
        """ 
        Returns True when IMU is setup properly, False if not.
        Checks the IMU is setup by ensuring that at least 25 IMU readings of 
        (acc, mag, gyro) data is not equal to 0. If this condition is satisfied, the
        function returns True. If the IMU continues reading 0s more than a count of 
        250 times, then this function returns False. 
        """
        imu_data = []
        count = 0
        if not self.is_sim: 
            while (len(imu_data) < 25): 
                count += 1 
                data = self.get_imu()
                if (data.get("acc") != 0 and data.get("mag") != 0 and data.get("gyro")!=0): 
                    imu_data.append(data)
                if (count > 250): 
                    return False
            return True 
