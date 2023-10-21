# pip install pyserial
# import serial
# arduinoSerialData = serial.Serial()
# arduinoSerialData.port = “COM4”
# arduinoSerialData.baudrate = 19200
# arduinoSerialData.timeout = 1
# arduinoSerialData.setDTR(False)
# #arduinoSerialData.setRTS(False)
# arduinoSerialData.open()
# while(True):
#     b = arduinoSerialData.readline().decode(‘utf-8’).strip().split(‘,’)
#     print(b)
import serial
import time

ser = serial.Serial("COM3", 115200)  # serial port


class Sensor_Data:
    def __init__(self):
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.mag_x = 0
        self.mag_y = 0
        self.mag_z = 0
        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0
        self.iteration = 0

    def set_IMU_all(self, cur_data):
        self.accel_x = cur_data[0]
        self.accel_y = cur_data[1]
        self.accel_z = cur_data[2]
        self.mag_x = cur_data[3]
        self.mag_y = cur_data[4]
        self.mag_z = cur_data[5]
        self.gyro_x = cur_data[6]
        self.gyro_y = cur_data[7]
        self.gyro_z = cur_data[8]
        self.iteration = cur_data[9]

    def print_data(self):
        print("a_x: " + str(self.accel_x))
        print("a_y: " + str(self.accel_y))
        print("a_z: " + str(self.accel_z))
        print("m_x: " + str(self.mag_x))
        print("m_y: " + str(self.mag_y))
        print("m_z: " + str(self.mag_z))
        print("g_x: " + str(self.gyro_x))
        print("g_y: " + str(self.gyro_y))
        print("g_z: " + str(self.gyro_z))


cur_data = []
readings = []


#
#
# def parse_reading(str):
#     c_index = str.index("c")
#     cur_data.append(float(str[4:c_index-1]))


def parse_whole_reading(str):
    s = Sensor_Data()
    a_x_index = str.index("a_x")
    a_y_index = str.index("a_y")
    a_z_index = str.index("a_z")
    m_x_index = str.index("m_x")
    m_y_index = str.index("m_y")
    m_z_index = str.index("m_z")
    g_x_index = str.index("g_x")
    g_y_index = str.index("g_y")
    g_z_index = str.index("g_z")
    c_index = str.index("c")
    grouping = []
    print("here2")
    grouping.append(str[a_x_index + 4 : a_y_index - 1])
    grouping.append(str[a_y_index + 4 : a_z_index - 1])
    grouping.append(str[a_z_index + 4 : m_x_index - 1])
    grouping.append(str[m_x_index + 4 : m_y_index - 1])
    grouping.append(str[m_y_index + 4 : m_z_index - 1])
    grouping.append(str[m_z_index + 4 : g_x_index - 1])
    grouping.append(str[g_x_index + 4 : g_y_index - 1])
    grouping.append(str[g_y_index + 4 : g_z_index - 1])
    grouping.append(str[g_z_index + 4 : c_index - 1])
    grouping.append(str[c_index + 2 :])

    s.set_IMU_all(grouping)
    readings.append(s)
    s.print_data()


# while True:
#     time.sleep(2)
#     try:
#         b = ser.readline()
#         str_b = str(b)
#         reading = str_b[2:-5]
#         if len(cur_data) == 0 and reading[0:3] != "a_x":
#             continue
#         else:
#             parse_reading(reading)
#             if len(cur_data) == 9:
#                 s = Sensor_Data()
#                 s.set_IMU_all(cur_data)
#                 readings.append(s)
#                 cur_data = []
#     except:
#         print("Keyboard Interrupt")
#         print(readings)
#         break


while True:
    try:
        b = ser.readline()
        str_b = str(b)
        reading = str_b[2:-5]
        parse_whole_reading(reading)

    except:
        print("Keyboard Interrupt")
        print(readings)
        break
