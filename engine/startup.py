import electrical.gps as gps 
import electrical.imu as imu 
import electrical.rf_module as rf_module
# TODO: documentation
class Startup():
    def execute_startup():
        #turn on radio and send radio commands 
        rf_module.startup() #TODO: complete function
        #initilize GPS 
        gps.startup() #TODO: connect gps startup function with rf_packet 
        #initialize IMU 
        imu.startup() #TODO: connect imu startup function with rf_packet
        
        