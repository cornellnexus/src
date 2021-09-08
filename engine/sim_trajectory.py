import numpy as np
import math
import matplotlib.pyplot as plt

from matplotlib import animation as animation
from matplotlib import patches as patch

from engine.grid import Grid
from engine.kinematics import limit_cmds, feedback_lin
from engine.pid_controller import PID
from engine.robot import Robot
from engine.mission import Mission

'''PLOTTING'''


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
        waypoints_arr[i, :] = np.asarray(waypoints[i].get_coords())
    return waypoints_arr


def get_plot_boundaries(meters_grid, delta):
    """
    Given some grid to be plotted, and a delta value, returns the desired 
    x limits and y limits for the plot.

    Arguments:
        meters_grid: a Grid object
        delta: the width of the border between
    Returns:
        xlim: a list containing the left and right boundaries of the grid, taking delta into account
        ylim: a list containing the top and bottom boundaries of the grid, taking delta into account
    """

    size = np.shape(meters_grid)
    min_coords = meters_grid[0, 0].get_coords()
    max_coords = meters_grid[size[0] - 1, size[1] - 1].get_coords()
    xlim = [min_coords[0] - delta, max_coords[0] + delta]
    ylim = [min_coords[1] - delta, max_coords[1] + delta]
    return xlim, ylim


if __name__ == "__main__":
    # Initialize robot
    r2d2 = Robot(0, 0, math.pi / 2, epsilon=0.2, max_v=0.5, radius=0.2, init_mode=2)

    ''' ------------------ SETUP ----------------------- '''
    # # Grid: Engineering Quad
    # g = Grid(42.444250, 42.444599, -76.483682, -76.483276)
    #
    # # Pass in 'full' to get full traversal path
    # waypoints = g.get_waypoints('borders')
    # goals = np.array(g.gps_waypoints)
    #
    # # Amount of simulated noise added to robot's state
    # NOISE_RANGE = .1
    #
    # # Larger epsilons means a larger turning radius
    # EPSILON = 0.2
    #
    # # Used in limit_cmds
    # MAX_V = 0.5
    # ROBOT_RADIUS = 0.2
    # ALLOWED_DIST_ERROR = 0.5  # was 0.5, weird when 10 and everything 0.1
    # TIME_STEP = 0.1  # is this the time by which we are going to sleep after each movement?
    #
    # loc_pid_x = PID(
    #     Kp=1, Ki=0, Kd=0, target=0, sample_time=TIME_STEP, output_limits=(None, None)
    # )
    #
    # loc_pid_y = PID(
    #     Kp=1, Ki=0, Kd=0, target=0, sample_time=TIME_STEP, output_limits=(None, None)
    # )

    m = Mission(r2d2)
    '''------------------- TRAVERSAL PHASE -------------------'''
    print(m.unvisited_goals)
    print("\n\n\n\n\n")
    m.execute_mission()
    print(m.unvisited_goals)

    ''' ---------- MISSION COMPLETE, PLOT TRUTH POSE --------------'''

    plt.style.use('seaborn-whitegrid')
    x_coords = m.robot.truthpose[:, 0]
    y_coords = m.robot.truthpose[:, 1]
    fig, ax = plt.subplots()
    ax.plot(x_coords, y_coords, '-b')
    ax.plot(x_coords[0], y_coords[0], 'gx')
    goals = waypoints_to_array(m.waypoints)
    ax.plot(goals[:, 0], goals[:, 1], 'rx')

    xbounds, ybounds = get_plot_boundaries(m.grid.meters_grid, 5)
    plt.xlim(xbounds)
    plt.ylim(ybounds)
    # plt.show()

    circle_patch = plt.Circle((5, 5), 1, fc="green")
    wedge_patch = patch.Wedge(
        (5, 1), 3, 100, 80, animated=True, fill=False, width=2, ec="g", hatch="xx"
    )


    def init():
        circle_patch.center = (0, 0)
        ax.add_patch(circle_patch)
        # ax.add_patch(arc_patch)
        ax.add_patch(wedge_patch)
        return circle_patch, wedge_patch


    def animate(i):
        x_coord = r2d2.truthpose[i, 0]
        y_coord = r2d2.truthpose[i, 1]
        circle_patch.center = (x_coord, y_coord)
        wedge_patch.update({"center": [x_coord, y_coord]})
        wedge_patch.theta1 = np.degrees(r2d2.truthpose[i, 2]) - 10
        wedge_patch.theta2 = np.degrees(r2d2.truthpose[i, 2]) + 10

        # print(wedge_patch.theta1, wedge_patch.theta2)
        # print(wedge_patch.center)
        return circle_patch, wedge_patch


    anim = animation.FuncAnimation(
        fig, animate, init_func=init, frames=np.shape(r2d2.truthpose)[0], interval=20, blit=True
    )

    plt.show()