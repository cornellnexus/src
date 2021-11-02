import serial
import time
import json
import ast

port = serial.Serial(port="/dev/tty.usbserial-017543DC", baudrate=57600, timeout=0)
port.flushInput()

while True:
  if port.in_waiting > 90:
    line = port.readline().decode("utf-8")
    # print(line)

    #----------------------Get dict from string------------------------

    if line[-1] == "\n" and line[0] == "{":
      data_dict = ast.literal_eval(line.rstrip("\n"))

    # data_dict = json.loads(line)
      print(data_dict)
    # time.sleep(1)
    #------------------------------------------------------------------
