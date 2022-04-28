import serial
import os
import sys


def get_path(folder):

    cwd = os.getcwd()
    sys.path.append(cwd + "/" + folder)
    return sys.path

# Testing Procedure: 

# 1. Initialize Radio modules (raspberry pi + computer). Ensure that data can be transmitted & received properly. 

# 2. Initialize GUI running at the same time  

# 3. Begin transmitting data from the raspberry pi’s radio module (read packets from a gui_to_rpi.csv) 
ser = serial.Serial("/dev/cu.usbserial-017543DC", 57600)
rpi_to_gui = open((get_path('csv')[-1] + '/gui_to_rpi.csv'), "r")  # open csv file of rpi to gui data

while True:
  packet = rpi_to_gui.readlines()[-1]  # get last line of csv file
  print("Sending the following packet: " + str(packet))
  ser.writelines(packet)
#   TODO: need to figure out when we want data to be transmitted in the code

# Now try in sim trajectory - uncomment # RPI_GUI_TEST lines in sim trajectory


# 4. Check if the GUI has updated as expected. 

