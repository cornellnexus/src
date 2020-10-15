# pip install pyserial

# import serial
# arduinoSerialData = serial.Serial()
# arduinoSerialData.port = "COM4"
# arduinoSerialData.baudrate = 19200
# arduinoSerialData.timeout = 1
# arduinoSerialData.setDTR(False)
# #arduinoSerialData.setRTS(False)
# arduinoSerialData.open()
# while(True):
#     b = arduinoSerialData.readline().decode('utf-8').strip().split(',')
#     print(b)


import serial
import time
ser = serial.Serial('COM3', 115200)  # serial port


class Sensor_Data:
    def __init__(self):
        IMU1_x = 0
        IMU1_y = 0
        IMU1_z = 0
        IMU2_x = 0
        IMU2_y = 0
        IMU2_z = 0
        IMU3_x = 0
        IMU3_y = 0
        IMU3_z = 0

    def set_IMU_all(self, cur_data):
        self.IMU1_x = cur_data[0]
        self.IMU2_x = cur_data[1]
        self.IMU3_x = cur_data[2]
        self.IMU1_y = cur_data[3]
        self.IMU2_y = cur_data[4]
        self.IMU3_y = cur_data[5]
        self.IMU1_z = cur_data[6]
        self.IMU2_z = cur_data[7]
        self.IMU3_z = cur_data[8]


cur_data = []
readings = []

while True:
    try:
        b = ser.readline()
        str_b = str(b)
        value = str_b[2:-5]
        cur_data.append(value)
        if len(cur_data) == 9:
            s = Sensor_Data()
            s.set_IMU_all(cur_data)
            readings.append(s)
            cur_data = []

        print(value)
    except:
        print("Keyboard Interrupt")
        print(readings)
        break
