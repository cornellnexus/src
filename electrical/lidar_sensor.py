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
            
                #print('Distance = ', bytes_serial[2]+bytes_serial[3]*256, ' -------- ', end = '')

                if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # check first two bytes
                    distance = bytes_serial[2] + bytes_serial[3]*256 # distance in next two bytes
                    strength = bytes_serial[4] + bytes_serial[5]*256 # signal strength in next two bytes
                    return distance


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

    def check_if_full(self, sample_rate, bucket_dist):
        '''
        Detects if the bin is full or not
        '''
        print('checking if full')
        init_time = time.time()
        counter = 0
        while time.time() < init_time + sample_rate*10:
                print('reading taken', counter)
            #try:
                new_obj_dist = self.get_lidar_data()
                if counter > 10*sample_rate:
                    return True #object detected 
                if new_obj_dist <= bucket_dist:
                    counter+=1
                else:
                    return False #false alarm
            #except:
                #print('THE EXCEPT IS HERE')
                #break

    
## testing code: 
if __name__ == "__main__":
    lidar = Lidar()
    sample_rate = 100 
    lidar.set_samp_rate(sample_rate) #configuring the sensor: sample rate and default baud rate 11000smth
    bucket_dist = 100 #in cm
    while True:
        #try:        
            distance = lidar.get_lidar_data() # read values
            print("Distance: ", distance)
            if distance <= bucket_dist:
                bucket_full = lidar.check_if_full(sample_rate, bucket_dist)
                print("bucket is full: ", bucket_full)
                if bucket_full == True:
                    break
        #except:
            #print("error")
            #break