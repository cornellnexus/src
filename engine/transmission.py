import logging
import time
import serial

format = "%(asctime)s: %(message)s"
logging.basicConfig(format=format, level=logging.INFO,
                    datefmt="%H:%M:%S")

# ser = serial.Serial("/dev/ttyS0", 57600) # Uncomment for RPI to GUI test
# ser = serial.Serial("/dev/cu.usbserial-017543DC", 57600) # RPI_GUI_TEST
def send_packet_to_gui(name, developer_flags, database, rpi_to_gui):
    logging.info("Thread %s: starting", name)
    while developer_flags.rpi_comms:
        packet = database.make_packet()
        if developer_flags.is_sim and developer_flags.should_store_data:
            # Simulate sending data packet to gui from rpi
            rpi_to_gui.write(str(packet) + '\n')
        elif not developer_flags.is_sim:
              # Sending data packet to gui from rpi
            cast_data = bytes(packet, encoding = 'utf-8') 
            ser = serial.Serial("/dev/ttyS0", 57600) # Uncomment for RPI to GUI test
            ser.write(cast_data)
        # logging.info("Sent packet: " + packet)
        time.sleep(0.01)
    logging.info("Thread %s: finishing", name)
    if developer_flags.is_sim and developer_flags.should_store_data:
        rpi_to_gui.close()