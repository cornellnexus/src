from radio_module import RadioSession
import serial
rs = RadioSession(serial.Serial(port="/dev/ttyS0", baudrate=57600, timeout=0))
rs.setup_robot()
