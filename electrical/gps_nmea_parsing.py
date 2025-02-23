import serial
import pynmea2


"""
This code is mainly for extracting pure longitude and latitude data from
a the serial port our ublox C099-F9P gps reciever is connected to.

Note that the C099-F9P module MUST be configured to output NMEA messages,
which may be of type GGC, RMC, etc.  
More on GPS messaging protocol can be read here: https://cdn.sparkfun.com/assets/f/7/4/3/5/PM-15136.pdf
"""

class GPSSerialReader:

    def __init__(self, port, br):
        # port is the physical port that the GPS module is plugged into on board
        # port starts at 0, on linux is of form '/dev/ttyS*' where * is the number referring to port
        self.port = port
        # higher baud rate is usually better for more accurate positioning
        self.baud_rate = br


    def read_serial(self):
        with serial.Serial(self.port, self.baud_rate, timeout=1) as ser:
            while True:
                line = ser.readline().decode('ascii', errors='replace')
                if line.startswith('$G'):
                    try:
                        msg = pynmea2.parse(line)
                        # Assuming the message is of type GGA or RMC which contains lat/long
                        print(f"Latitude: {msg.latitude}, Longitude: {msg.longitude}")
                    except pynmea2.ParseError as e:
                        print(f"Parse error: {e}")