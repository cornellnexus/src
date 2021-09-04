""" This module receives the serially transmitted data from the rf module"""

import sys
import time
import serial
import difflib
import pigpio
from gpio import *

tx = rf_tx
rx = rf_rx

# Ground Station 
ser_gnd = serial.Serial(port="computer port", 9600, timeout=0)


def gnd_receive():
    while ser_gnd.inWaiting():
        received_data_gnd = ser_gnd.readline()
        print(received_data)
        time.sleep(0.2)


def gnd_transmit(data):
    ser_gnd.write(data)


# RPi Station

def rpi_receive():
    '''
    input data to rpi by wrapping gpio pins as uart port.
    '''
    try:
        pi = pigpio.pi()
        pi.set_mode(rx, pigpio.INPUT)
        pi.bb_serial_read_open(rx, 9600, 8)

        print("data - software serial: ")
        while True:
            (count, rf_data) = pi.bb_serial_read(rx)
            if count:
                print
                rf_data
        time.sleep(1)

except:
pi.bb_serial_read_close(rx)
pi.stop()

# ser_rpi = serial.Serial(port = "rpi port", 9600, timeout = 0)

# def rpi_receive(): 
#   while ser_rpi.inWaiting():  
#       received_data_rpi = ser_rpi.readline()
#       print(received_data)
#       time.sleep(0.2)

# def rpi_transmit(data):
#   ser_rpi.write(data)
