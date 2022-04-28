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
ser = serial.Serial("/dev/ttyS0", 57600)
rpi_to_gui = open((get_path('tests')[-1] + '/functionality_tests/csv/gui_to_rpi.csv'), "r")  # open csv file of rpi to gui data

while True:
  packet = "phase:0;p_weight:0.0;acc:0.01,0.01,0.01;n_dist:00.0;rot:00.00;last_n:000.00,000.00;vel:0.00;next_n:000.00,000.00;coord:000.00,000.00;batt:000;ctrl:1" + "\n"
  # basic packet
#   packet = str(rpi_to_gui.readlines()[-1]) + "\n" # get last line of csv file

  cast_data = bytes(packet, encoding = 'utf-8') 
  print("Sending the following packet: " + str(packet))
  ser.write(cast_data)
#   TODO: need to figure out when we want data to be transmitted in the code

# Now try in sim trajectory - uncomment # RPI_GUI_TEST lines in sim trajectory


# 4. Check if the GUI has updated as expected. 




# Run gui_to_rpi.py on raspberry pi, sending just a basic packet
# Run retrieve_inputs on personal laptop, monitor prints as well as robot_data.csv
