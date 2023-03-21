import serial
import time

'''
Handshake script to be run by the robot. Should be run slightly before running the laptop handshake script.
Establishes a handshake by decoding a message sent by the computer and then sends a message to the laptop
to make a two way connection.
'''

# Initializes port and clears any exisiting messages
port = serial.Serial(port="/dev/serial0", baudrate=57600, timeout=0)
port.flush()
port.flushInput()
print("RPI script begins")

# Send message to laptop to start handshake
stringy = "robot_to_computer"

# TODO: Check if casting to bytes / encoding is necessary
cast_stringy = bytes(stringy, encoding = "utf-8")
port.write(cast_stringy)
print("RPI script done")