"""
This is a test script for local computers. Run the setup_basestation()

"""

from electrical.radio_module import RadioSession
import serial
rs = RadioSession(serial.Serial(port="/dev/tty.usbserial-017543DC", baudrate=57600, timeout=0))
rs.setup_basestation()

