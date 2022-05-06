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
  #packet = "phase:3;p_weight:02.0;acc:3.01,4.01,5.01;n_dist:60.0;rot:07.00;last_n:008.00,008.00;vel:9.00;next_n:010.00,011.00;coord:030.00,015.00;batt:013;ctrl:3" + "\n"
  packet = "phase:4;p_weight:02.0;acc:0.00,4.00,2.00;n_dist:40.0;rot:02.00;last_n:008.00,008.00;vel:9.00;next_n:010.00,011.00;coord:030.00,015.00;batt:013;ctrl:3" + "\n"
  cast_data = bytes(packet, encoding = 'utf-8') 
  print("Sending the following packet: " + str(packet))
  ser.write(cast_data)

# 4. Check if the GUI has updated as expected. 