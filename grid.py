from node import Node
import numpy as np
import math
import serial
import geopy.distance
import matplotlib.pyplot as plt
from math import log
import time


class Grid:
    """
    Instances represent the current state of the graph of the robot's traversal.

    INSTANCE ATTRIBUTES:
        traversal_path: Ordered list of Node objects to travel to [Node list]

        nodes_dict: Ordered dictionary of Nodes to travel to where keys are (x,y) 
                   tuples and values are Node objects

        # obstacle_coordinates: List of Node objects that represent obstacle nodes

        long_min: User input of minimum longitude boundary point
        long_max: User input of maximum longitude boundary point
        lat_min: User input of minimum latitude boundary point
        lat_max: User input of maximum latitude boundary point
    """

    def __init__(self, lat_min, lat_max, long_min, long_max):
        self.traversal_path = []
        self.nodes_dict = {}
        self.obstacle_coordinates = []

        self.long_min = long_min
        self.long_max = long_max
        self.lat_min = lat_min
        self.lat_max = lat_max

        self.obstacle_length_limit = 10

        def calc_step(long_min, long_max, lat_min, lat_max):
            long_range = geopy.distance.distance(
                (long_min, lat_min), (long_max, lat_min)).meters
            lat_range = geopy.distance.distance(
                (long_min, lat_min), (long_min, lat_max)).meters

            # Underestimate to achieve integer number of rows and columns
            num_long_steps = int(long_range // 1.8288)
            num_lat_steps = int(lat_range // 1.8288)

            long_step = (self.long_max - self.long_min) / num_long_steps
            lat_step = (self.lat_max - self.lat_min) / num_lat_steps
            return long_step, lat_step, num_long_steps+1, num_lat_steps+1

        def generate_nodes(start_long, start_lat, row_num, col_num, long_step, lat_step):
            origin = (start_long, start_lat)
            traversal_path = []
            nodes_dict = {}

            true_min_lat = start_lat
            true_max_lat = start_lat + (row_num - 1) * lat_step

            for i in range(col_num):
                for j in range(row_num):
                    if i % 2 == 0:
                        node = Node(origin[0] + i*long_step,
                                    origin[1] + j*lat_step)
                        traversal_path.append(node)
                        nodes_dict[node.get_coords()] = node

                    elif i % 2 == 1:
                        long_pos = origin[0] + i*long_step
                        lat_pos = origin[1] + \
                            ((row_num - 1) * lat_step) - j*lat_step
                        node = Node(long_pos, lat_pos)
                        traversal_path.append(node)
                        nodes_dict[node.get_coords()] = node
            return traversal_path, nodes_dict, true_min_lat, true_max_lat

        self.long_step, self.lat_step, self.col_num, self.row_num = calc_step(
            long_min, long_max, lat_min, lat_max)
        self.traversal_path, self.nodes_dict, self.true_min_lat, self.true_max_lat = generate_nodes(
            long_min, lat_min, self.row_num, self.col_num, self.long_step, self.lat_step)


# g = Graph(42.444496,42.444543, -76.488495, -76.488419)
# print(g.traversal_path)

# testlist = []
# xlist = []
# ylist = []

# for node in g.traversal_path:
#     coords = node.get_coords()
#     xlist.append(coords[0])
#     ylist.append(coords[1])

# plt.plot(xlist, ylist, marker='o', markerfacecolor='blue')
# plt.ylim(-76.488500,-76.488400)
# plt.xlim(42.444444,42.444590)
# plt.show()

# g = Graph(42.4596, 42.4642, -76.5119, -76.5013)

# testlist = []
# xlist = []
# ylist = []

# for node in g.traversal_path:
#     coords = node.get_coords()
#     xlist.append(coords[0])
#     ylist.append(coords[1])

# plt.plot(xlist, ylist, marker='o', markerfacecolor='blue')
# plt.ylim(-76.5149, -76.5000)
# plt.xlim(42.4580, 42.4650)# g = Graph(42.444496,42.444543, -76.488495, -76.488419)
