import numpy as np
import math
import matplotlib.pyplot as plt

from matplotlib import animation as animation
from matplotlib import patches as patch
from constants.definitions import CSV_PATH

from engine.robot import Robot
from engine.robot import Phase
from engine.base_station import BaseStation
from engine.mission import Mission
from engine.mission import ControlMode
from engine.database import DataBase

import logging
import threading
import time

import sys
import os
import serial

# ser = serial.Serial("/dev/ttyS0", 57600) # Uncomment for RPI to GUI test

def waypoints_to_array(waypoints):
    """
    Arguments:
        waypoints: a list of Node objects

    Returns:
        waypoints_arr: a 1D np array of coordinates, each of which corresponds to a waypoint in waypoints
    """

    n = len(waypoints)
    waypoints_arr = np.empty([n, 2])
    for i in range(n):
        waypoints_arr[i, :] = np.asarray(waypoints[i].get_m_coords())
    return waypoints_arr


def get_plot_boundaries(nodes, delta):
    """
    Given some grid to be plotted, and a delta value, returns the desired 
    x limits and y limits for the plot.

    Arguments:
        nodes: a Grid object
        delta: the width of the border between
    Returns:
        xlim: a list containing the left and right boundaries of the grid, taking delta into account
        ylim: a list containing the top and bottom boundaries of the grid, taking delta into account
    """

    size = np.shape(nodes)
    min_coords = nodes[0, 0].get_m_coords()
    max_coords = nodes[size[0] - 1, size[1] - 1].get_m_coords()
    xlim = [min_coords[0] - delta, max_coords[0] + delta]
    ylim = [min_coords[1] - delta, max_coords[1] + delta]
    return xlim, ylim

if __name__ == "__main__":
    global rpi_comms, is_sim, is_store
    rpi_comms = False # Set to true when the rpi/robot is communicating w/ the GUI
    is_sim = True # Set to true when simulating the rpi, set to false when running on rpi
    is_store = False # Set to true when we want to track/store csv data
    format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO,
                        datefmt="%H:%M:%S")

    if is_sim and is_store:
        # open csv file of rpi to gui data
        rpi_to_gui = open(
            (CSV_PATH + '/rpi_to_gui_simulation.csv'), "a")

    # ser = serial.Serial("/dev/cu.usbserial-017543DC", 57600) # RPI_GUI_TEST
    r2d2 = Robot(0, 0, math.pi / 4, epsilon=0.2, max_v=0.5,
                 radius=0.2, init_phase=Phase.TRAVERSE, 
                 is_sim=is_sim, is_store=is_store)

    base_r2d2 = BaseStation((42.444250, -76.483682))
    database = DataBase(r2d2)
    m = Mission(robot=r2d2, base_station=base_r2d2,
                init_control_mode=ControlMode.LAWNMOWER)

    def send_packet_to_gui(name):
        logging.info("Thread %s: starting", name)
        while rpi_comms:
            packet = database.make_packet()
            if is_sim and is_store:
                # Simulate sending data packet to gui from rpi
                rpi_to_gui.write(str(packet) + '\n')
            elif not is_sim:
                 # Sending data packet to gui from rpi
                cast_data = bytes(packet, encoding = 'utf-8') 
                ser.write(cast_data)
            # logging.info("Sent packet: " + packet)
            time.sleep(0.01)
        logging.info("Thread %s: finishing", name)
        if is_sim and is_store:
            rpi_to_gui.close()

    '''------------------- MISSION EXECUTION -------------------'''
    packet_sender = threading.Thread(target=send_packet_to_gui, args=(
        1,), daemon=True)  # Thread to read and send robot properties
    packet_sender.start()

    m.execute_mission(database)  # Run main mission

    # once gui.gui.py is closed, also close gui.retrieve_inputs.py
    os.system("pkill -f gui.retrieve_inputs")

    ''' ---------- MISSION COMPLETE, PLOT TRUTH POSE --------------'''

    plt.style.use('seaborn-whitegrid')
    x_coords = m.robot.truthpose[:, 0]
    y_coords = m.robot.truthpose[:, 1]
    fig, ax = plt.subplots()
    ax.plot(x_coords, y_coords, '-b')
    ax.plot(x_coords[0], y_coords[0], 'gx')
    margin = 5
    if m.control_mode == ControlMode.ROOMBA:
        range = m.roomba_radius + margin
        init_x = m.base_station_loc[0]
        init_y = m.base_station_loc[1]
        plt.xlim([init_x-range, init_x+range])
        plt.ylim([init_y-range, init_y+range])
        circle = plt.Circle((init_x, init_y), m.roomba_radius)
        ax.add_patch(circle)

    elif m.control_mode != ControlMode.MANUAL:
        goals = waypoints_to_array(m.all_waypoints)
        active_nodes = waypoints_to_array(m.active_waypoints)
        inactive_nodes = waypoints_to_array(m.inactive_waypoints)
        ax.plot(active_nodes[:, 0], active_nodes[:, 1], 'bx')
        ax.plot(inactive_nodes[:, 0], inactive_nodes[:, 1], 'rx')
        xbounds, ybounds = get_plot_boundaries(m.grid.nodes, margin)
        plt.xlim(xbounds)
        plt.ylim(ybounds)

    circle_patch = plt.Circle((5, 5), 1, fc="green")
    wedge_patch = patch.Wedge(
        (5, 1), 3, 100, 80, animated=True, fill=False, width=2, ec="g", hatch="xx"
    )
    # Plot base station:
    circle_patch_base = plt.Circle((5, 5), 1, fc="red")
    # The heading of base station in degrees
    base_angle_degrees = math.degrees(m.base_station_angle)
    wedge_patch_base = patch.Wedge(
        m.base_station_loc, 3, base_angle_degrees-10, base_angle_degrees+10, fill=False, width=2, ec="r", hatch="xx"
    )

    def init():
        circle_patch.center = (0, 0)
        circle_patch_base.center = m.base_station_loc
        ax.add_patch(circle_patch)
        ax.add_patch(wedge_patch)
        ax.add_patch(circle_patch_base)
        ax.add_patch(wedge_patch_base)
        return circle_patch, wedge_patch

    def animate(i):
        x_coord = m.robot.truthpose[i, 0]
        y_coord = m.robot.truthpose[i, 1]
        circle_patch.center = (x_coord, y_coord)
        wedge_patch.update({"center": [x_coord, y_coord]})
        wedge_patch.theta1 = np.degrees(m.robot.truthpose[i, 2]) - 10
        wedge_patch.theta2 = np.degrees(m.robot.truthpose[i, 2]) + 10
        return circle_patch, wedge_patch

    anim = animation.FuncAnimation(
        fig, animate, init_func=init, frames=np.shape(m.robot.truthpose)[0], interval=20, blit=True
    )

    plt.show()
