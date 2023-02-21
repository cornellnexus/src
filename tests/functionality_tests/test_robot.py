"""
This is a test script for the raspberry pi. Run the setup_robot()

"""
from electrical.radio_module import RadioModule
import serial
rs = RadioModule(serial.Serial(port="/dev/ttyS0", baudrate=57600, timeout=0))
rs.setup_robot()
