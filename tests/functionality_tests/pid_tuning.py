import math

from electrical.motor_controller import MotorController
from engine.grid import Grid
from engine.database import DataBase
from engine.robot import Robot, Phase

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import time
from electrical.motor_controller import PidGpio
from electrical.gps import GPS
from electrical.imu import IMU

import serial
import busio
import board

import csv


# def traverse_straight_line(lat_min=42.444250, lat_max=42.444599, long_min=-76.483682, long_max=-76.483276, allowed_dist_error=0.05):
#     """
#     Calls the robot to traverse in a straight line.
# 
#     Note that the default values for lat_min, lat_max, long_min, and long_max correspond
#     to the Eng Quad.
#     """
#     grid = Grid(lat_min, lat_max, long_min, long_max)
#     # Default parameters for the robot initialized at position (0,0)
#     # max_v, heading, epsilon will likely be changed as we physically test
#     r2d2 = Robot(0, 0, math.pi / 4, epsilon=0.2, max_v=0.5,
#                  radius=0.2, init_phase=Phase.TRAVERSE)
# <<<<<<< HEAD
#     motor_controller = MotorController(r2d2)
#     motor_controller.setup()
# =======
#     motor_controller = PidGpio()
#     # gps = GPS(serial.Serial(port = "/dev/ttyACM0", baudrate = 9600, timeout = 1), False)
#     # gps.setup()
#     # init_gps = (gps.get_gps()["long"], gps.get_gps()["lat"])
#     gps = None
#     init_gps = None
#     imu = IMU(busio.I2C(board.SCL, board.SDA), False)
#     imu.setup()
# >>>>>>> 24f2fb4084cdde357f9b41f27a8564c9b4fd0472
#     database = DataBase(r2d2)
#     waypoints = grid.get_straight_line_waypoints(y_start_pct=0.5)
# 
#     # X-axis of the graph is the time step, Y-axis of graph is linear velocity
#     style.use('fivethirtyeight')
#     fig = plt.figure()
#     ax1 = fig.add_subplot(1, 1, 1)

#     def animate(i):
#         graph_data = open('csv/velocities.csv', 'r').read()
#         lines = graph_data.split('\n')
#         xs = []
#         ys = []
#         print(len(waypoints))
#         for line in lines:
#             if len(line) > 1:
#                 x, y = line.split(',')
#                 xs.append(float(x))
#                 ys.append(float(y))
#         ax1.clear()
#         ax1.plot(xs, ys)
# 
#     # clear the contents of the csv file
#     with open('csv/velocities.csv', 'w'):
#         pass
# 
#     ani = animation.FuncAnimation(fig, animate, interval=5)
# 
#     # traverses the waypoints, writes relevant data to csv
# 
#     time.sleep(2)
#     iter = 0
#     while len(waypoints) > 0:
#         curr_waypoint = waypoints[0].get_m_coords()
#         r2d2.move_to_target_node(
# <<<<<<< HEAD
#             curr_waypoint, allowed_dist_error, database, motor_controller)
# =======
#             curr_waypoint, allowed_dist_error, database, motor_controller, gps, init_gps, imu)
# >>>>>>> 24f2fb4084cdde357f9b41f27a8564c9b4fd0472
# 
#         with open('csv/velocities.csv', 'a') as f:
#             writer = csv.writer(f)
#             row = [iter, r2d2.linear_v]
#             writer.writerow(row)
#         waypoints.pop(0)
#         iter += 1
#         plt.pause(0.05)
#     plt.show()


traverse_straight_line()
