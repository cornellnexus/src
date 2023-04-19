import logging
import time
import serial
from engine.phase import Phase

format = "%(asctime)s: %(message)s"
logging.basicConfig(format=format, level=logging.INFO,
                    datefmt="%H:%M:%S")

# ser = serial.Serial("/dev/ttyS0", 57600) # Uncomment for RPI to GUI test
# ser = serial.Serial("/dev/cu.usbserial-017543DC", 57600) # RPI_GUI_TEST
def send_packet_to_gui(name, is_sim, robot_state, database):
    """
    Composes a data packet representing attributes of the current robot 
    (and in the future mission state) and transmits the data for the GUI. Currently,
    assumes there is a GUI listening for data packets.

    Arguments:
        name: int identifier for the thread
        is_sim: boolean flag on whether this is a simulation
        robot_state: Robot State object 
        database: Database object # TODO replace
    """
    logging.info("Thread %s: starting", name)
    while robot_state.phase != Phase.COMPLETE:
        packet = database.make_packet() # TODO: Replace implementation
        if not is_sim:
            # Sending data packet to gui from rpi
            cast_data = bytes(packet, encoding = 'utf-8') 
            ser = serial.Serial("/dev/ttyS0", 57600) # Uncomment for RPI to GUI test
            ser.write(cast_data)
        # logging.info("Sent packet: " + packet)
        time.sleep(0.01)
    logging.info("Thread %s: finishing", name)