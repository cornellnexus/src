import serial
import os
import sys

from electrical.radio_module import RadioModule
import serial
ser = serial.Serial("/dev/tty.usbserial-017543DC", 57600)


while True:
  print("attempting to read packet")
  packet = ser.readline().decode('utf-8')
  print("packet is " + packet)