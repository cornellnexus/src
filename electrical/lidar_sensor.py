"""Programming the LIDAR Sensor"""
import serial
import time

class Lidar:
    """ 
    This lidar class contains data on distance between sensor and an object
    """
    def __init__(self):
        #TODO: change address of serial device, this is a placeholder
        self.ser = serial.Serial("/dev/serial0", 115200,timeout=0) 
    
    def get_lidar_data(self):
        while True:
            counter = self.ser.in_waiting # count the number of bytes of the serial port
            if counter > 8:
                bytes_serial = self.ser.read(9) # read 9 bytes
                self.ser.reset_input_buffer() # reset buffer

            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # check first two bytes
                distance = bytes_serial[2] + bytes_serial[3]*256 # distance in next two bytes
                strength = bytes_serial[4] + bytes_serial[5]*256 # signal strength in next two bytes
                return distance/100.0,strength


    def check_data_reliability(self, strength):
        if strength < 100 or strength == 65535:
            return False
        else:
            return True

    
    def set_samp_rate(self, samp_rate = 100): # should it just be samp_rate in the parenthesis here?
        '''
        Change the sample rate (number of readings per second, Hz).
        Currently sets the sample rate to 100Hz. 
        '''
        samp_rate_packet = [0x5a,0x06,0x03,samp_rate,00,00] # sample rate byte array
        self.ser.write(samp_rate_packet) # send sample rate instruction
        time.sleep(0.1) # wait for change to take effect
        return

    def check_if_full(self, sample_rate, alert_dist):
        '''
        Detects if the bin is full or not
        '''
        init_time = time.time()
        counter = 0
        while time.time() < init_time + sample_rate*10:
            new_obj_dist = get_lidar_data()
            if new_obj_dist < alert_dist:
                counter+=1
            else:
                break
            if counter > 10*sample_rate:
                return True #object detected 

    #configuring the sensor: sample rate and default baud rate 11000smth
    sample_rate = 100
    set_samp_rate(sample_rate)
    alert_dist = 100 #in cm
    #t0 = time.time() # for timing
    dist_array = [] # for storing values

    while True:
        try:
            distance,strength = get_lidar_data() # read values
            print('Distance: {0:2.2f} m, Strength: {1:2.0f} / 65535 (16-bit)'.format(distance,strength))
            if distance < alert_dist:
                check_if_full(sample_rate, alert_dist)
        except:
            continue

    '''
    ALERT_DIST = Detects obstruction (less than the width of the box)
    while True:
        dist = get_object_distance 
        if dist < ALERT_DIST: 
            init_time = time.time()
            counter = 0
            for time.time() < init_time + 5: 
                new_dist_obj = get_object_distance 
                if new_dist_obj <  dist < ALERT_DIST: 
                    counter ++ 
            if counter > 10: 
                object is detected 
    '''
## testing code: 
# lidar_sensor = Lidar()
# data = lidar_sensor.get_lidar_data()