import math

from engine.grid import Grid
from engine.database import DataBase
from engine.robot import Robot, Phase

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

import csv


def traverse_straight_line(lat_min=42.444250, lat_max=42.444599, long_min=-76.483682, long_max=-76.483276, allowed_dist_error=0.1):
    grid = Grid(lat_min, lat_max, long_min, long_max)
    r2d2 = Robot(0, 0, math.pi / 4, epsilon=0.2, max_v=0.5,
                 radius=0.2, init_phase=Phase.TRAVERSE)
    database = DataBase(r2d2)
    waypoints = grid.get_straight_line_waypoints(y_start_pct=0.5)

    style.use('fivethirtyeight')
    fig = plt.figure()
    ax1 = fig.add_subplot(1, 1, 1)

    def animate(i):
        graph_data = open('csv/velocities.csv', 'r').read()
        lines = graph_data.split('\n')
        xs = []
        ys = []
        for line in lines:
            if len(line) > 1:
                x, y = line.split(',')
                xs.append(float(x))
                ys.append(float(y))
        ax1.clear()
        ax1.plot(xs, ys)

    iter = 0

    # clear the contents of the csv file
    with open('csv/velocities.csv', 'w'):
        pass

    while len(waypoints) > 0:
        curr_waypoint = waypoints[0].get_m_coords()
        r2d2.move_to_target_node(
            curr_waypoint, allowed_dist_error, database)

        with open('csv/velocities.csv', 'a') as f:
            writer = csv.writer(f)
            row = [iter, r2d2.linear_v]
            print(row)
            writer.writerow(row)
        waypoints.pop(0)
        iter += 1
    ani = animation.FuncAnimation(fig, animate, interval=5)
    plt.show()


traverse_straight_line()
