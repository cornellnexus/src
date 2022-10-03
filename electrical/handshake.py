import serial
import time
import json
import ast
port = serial.Serial(port="/dev/tty.usbserial-017543DC", baudrate=57600, timeout=0)
port.flush()
port.flushInput()
t_end = time.time() + 5
line = None
while time.time() < t_end:
    if port.in_waiting > 0: 
        line = port.readline().decode("utf-8")
        break
stringy = "handshake_computer"
cast_stringy = bytes(stringy, encoding = "utf-8")
port.write(cast_stringy)

if (line == "handshake_robot"):
    t_end = time.time() + 5
    line = None
    while time.time() < t_end:
        if port.in_waiting > 0: 
            line = port.readline().decode("utf-8")
            break
    if(line == "Success"):
        print("Success")
    else:
        print("Failure")
    success = "Success"
    cast_success = bytes(success, encoding = "utf-8")
    port.write(cast_success)

else:
    print("Failure")