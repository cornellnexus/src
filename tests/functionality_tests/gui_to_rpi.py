import serial
import os
import sys

""""
This is the test script run on the raspberry pi to send over a hard-coded telemetry packet to be 
updated on the other device's (PC's) GUI window.
"""
# Testing Procedure: 

# 1. Initialize Radio modules (raspberry pi + PC). Ensure that data can be transmitted & received properly (turn them on.)

# 2. Initialize GUI running at the same time on PC

# 3. Begin transmitting data from the raspberry pi’s radio module (read packets from a gui_to_rpi.csv) 
ser = serial.Serial("/dev/ttyS0", 57600)

while True:
  packet = "phase:1;p_weight:1.0;acc:0.01,0.01,0.01;n_dist:00.0;rot:00.00;last_n:000.00,000.00;vel:0.00;next_n:000.00,000.00;coord:000.00,000.00;batt:000;ctrl:1" + "\n"
  cast_data = bytes(packet, encoding = 'utf-8') 
  print("Sending the following packet: " + str(packet))
  ser.write(cast_data)

# 4. Check if the GUI has updated as expected. 