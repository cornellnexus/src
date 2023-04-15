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

    # once gui.gui.py is closed, also close gui.retrieve_inputs.py
    os.system("pkill -f gui.retrieve_inputs")

    ''' ---------- MISSION COMPLETE, PLOT TRUTH POSE --------------'''
    plot_sim_traj(m=m) #  Plot the trajectory of the completed mission

    # def waypoints_to_array(waypoints):
    #     """
    #     Arguments:
    #         waypoints: a list of Node objects

    #     Returns:
    #         waypoints_arr: a 1D np array of coordinates, each of which corresponds to a waypoint in waypoints
    #     """

    #     n = len(waypoints)
    #     waypoints_arr = np.empty([n, 2])
    #     for i in range(n):
    #         waypoints_arr[i, :] = np.asarray(waypoints[i].get_m_coords())
    #     return waypoints_arr


    # def get_plot_boundaries(nodes, delta):
    #     """
    #     Given some grid to be plotted, and a delta value, returns the desired 
    #     x limits and y limits for the plot.

    #     Arguments:
    #         nodes: a Grid object
    #         delta: the width of the border between
    #     Returns:
    #         xlim: a list containing the left and right boundaries of the grid, taking delta into account
    #         ylim: a list containing the top and bottom boundaries of the grid, taking delta into account
    #     """

    #     size = np.shape(nodes)
    #     min_coords = nodes[0, 0].get_m_coords()
    #     max_coords = nodes[size[0] - 1, size[1] - 1].get_m_coords()
    #     xlim = [min_coords[0] - delta, max_coords[0] + delta]
    #     ylim = [min_coords[1] - delta, max_coords[1] + delta]
    #     return xlim, ylim


    # plt.style.use('seaborn-whitegrid')
    # x_coords = m.mission_state.robot.robot_state.truthpose[:, 0]
    # y_coords = m.mission_state.robot.robot_state.truthpose[:, 1]
    # fig, ax = plt.subplots()
    # ax.plot(x_coords, y_coords, '-b')
    # ax.plot(x_coords[0], y_coords[0], 'gx')
    # margin = 5
    # if m.mission_state.control_mode == ControlMode.ROOMBA:
    #     range = m.mission_state.roomba_radius + margin
    #     init_x = m.mission_state.base_station.position[0]
    #     init_y = m.mission_state.base_station.position[1]
    #     plt.xlim([init_x-range, init_x+range])
    #     plt.ylim([init_y-range, init_y+range])
    #     circle = plt.Circle((init_x, init_y), m.mission_state.roomba_radius)
    #     ax.add_patch(circle)

    # elif m.mission_state.control_mode != ControlMode.MANUAL:
    #     goals = waypoints_to_array(m.mission_state.all_waypoints)
    #     active_nodes = waypoints_to_array(m.mission_state.active_waypoints)
    #     inactive_nodes = waypoints_to_array(m.mission_state.inactive_waypoints)
    #     ax.plot(active_nodes[:, 0], active_nodes[:, 1], 'bx')
    #     ax.plot(inactive_nodes[:, 0], inactive_nodes[:, 1], 'rx')
    #     xbounds, ybounds = get_plot_boundaries(m.mission_state.grid.nodes, margin)
    #     plt.xlim(xbounds)
    #     plt.ylim(ybounds)

    # circle_patch = plt.Circle((5, 5), 1, fc="green")
    # wedge_patch = patch.Wedge(
    #     (5, 1), 3, 100, 80, animated=True, fill=False, width=2, ec="g", hatch="xx"
    # )
    # # Plot base station:
    # circle_patch_base = plt.Circle((5, 5), 1, fc="red")
    # # The heading of base station in degrees
    # base_angle_degrees = math.degrees(m.mission_state.base_station.heading)
    # wedge_patch_base = patch.Wedge(
    #     m.mission_state.base_station.position, 3, base_angle_degrees-10, base_angle_degrees+10, fill=False, width=2, ec="r", hatch="xx"
    # )

    # def init():
    #     circle_patch.center = (0, 0)
    #     circle_patch_base.center = m.mission_state.base_station.position
    #     ax.add_patch(circle_patch)
    #     ax.add_patch(wedge_patch)
    #     ax.add_patch(circle_patch_base)
    #     ax.add_patch(wedge_patch_base)
    #     return circle_patch, wedge_patch

    # def animate(i):
    #     x_coord = m.mission_state.robot.robot_state.truthpose[i, 0]
    #     y_coord = m.mission_state.robot.robot_state.truthpose[i, 1]
    #     circle_patch.center = (x_coord, y_coord)
    #     wedge_patch.update({"center": [x_coord, y_coord]})
    #     wedge_patch.theta1 = np.degrees(m.mission_state.robot.robot_state.truthpose[i, 2]) - 10
    #     wedge_patch.theta2 = np.degrees(m.mission_state.robot.robot_state.truthpose[i, 2]) + 10
    #     return circle_patch, wedge_patch

    # anim = animation.FuncAnimation(
    #     fig, animate, init_func=init, frames=np.shape(m.mission_state.robot.robot_state.truthpose)[0], interval=20, blit=True
    # )

    # plt.show()
