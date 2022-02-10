import numpy as np
import math
import matplotlib.pyplot as plt

from matplotlib import animation as animation
from matplotlib import patches as patch

from engine.robot import Robot
from engine.robot import Phase
from engine.base_station import BaseStation
from engine.mission import Mission
from engine.mission import ControlMode


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
    r2d2 = Robot(0, 0, math.pi / 4, epsilon=0.2, max_v=0.5, radius=0.2, init_phase=Phase.TRAVERSE)
    base_r2d2 = BaseStation((42.444250, -76.483682))
    m = Mission(robot=r2d2, base_station=base_r2d2, init_control_mode=ControlMode.LAWNMOWER_BORDERS)

    '''------------------- MISSION EXECUTION -------------------'''
    m.execute_mission()

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
        ax.plot(goals[:, 0], goals[:, 1], 'rx')

        xbounds, ybounds = get_plot_boundaries(m.grid.nodes, margin)
        plt.xlim(xbounds)
        plt.ylim(ybounds)

    circle_patch = plt.Circle((5, 5), 1, fc="green")
    wedge_patch = patch.Wedge(
        (5, 1), 3, 100, 80, animated=True, fill=False, width=2, ec="g", hatch="xx"
    )

    # Plot base station:
    circle_patch_base = plt.Circle((5, 5), 1, fc="red")
    base_angle_degrees = math.degrees(m.base_station_angle)  # The heading of base station in degrees
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
