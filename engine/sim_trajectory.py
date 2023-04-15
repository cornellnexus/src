import numpy as np
import math
import matplotlib.pyplot as plt

from matplotlib import animation as animation
from matplotlib import patches as patch
from constants.definitions import CSV_PATH

from engine.robot import Robot
from engine.robot_state import Robot_State
from engine.robot import Phase
from engine.mission import Mission
from engine.mission_state import Mission_State
from engine.control_mode import ControlMode
from engine.database import DataBase
from engine.transmission import send_packet_to_gui
from engine.is_raspberrypi import is_raspberrypi

from engine.plot_trajectory import plot_sim_traj


import threading
import os

class Flags:
    def __init__(self):
        self.rpi_comms = False # Set to true when the rpi/robot is communicating w/ the GUI
        self.is_sim = not is_raspberrypi() # Set to true when simulating the rpi, set to false when running on rpi
        self.should_store_data = False # Set to true when we want to track/store csv data

if __name__ == "__main__":
    # rpi_comms = False # Set to true when the rpi/robot is communicating w/ the GUI
    # is_sim = not is_raspberrypi() # Set to true when simulating the rpi, set to false when running on rpi
    # should_store_data = False # Set to true when we want to track/store csv data
    developer_flags = Flags()
    rpi_to_gui = None

    if developer_flags.is_sim and developer_flags.should_store_data:
        # open csv file of rpi to gui data
        rpi_to_gui = open(
            (CSV_PATH + '/rpi_to_gui_simulation.csv'), "a")

    r2d2_state = Robot_State(xpos=0, ypos=0, heading=math.pi / 4, epsilon=0.2, max_velocity=0.5, radius=0.2, phase = Phase.TRAVERSE)
    r2d2 = Robot(robot_state=r2d2_state)
    database = DataBase(r2d2) # TODO: Replace w new packet transmission impl
    mission_state = Mission_State(robot=r2d2, base_station_coord=(42.444250, -76.483682),
                init_control_mode=ControlMode.LAWNMOWER)
    m = Mission(mission_state=mission_state)

    '''------------------- MISSION EXECUTION -------------------'''
    packet_sender = threading.Thread(target=send_packet_to_gui, args=(
        "1", developer_flags, database, rpi_to_gui), daemon=True)  # Thread to read and send robot properties
    packet_sender.start()

    m.execute_mission(database)  # Run main mission

    ''' ---------- MISSION COMPLETE, PLOT TRUTH POSE --------------'''
    plot_sim_traj(m=m) #  Plot the trajectory of the completed mission