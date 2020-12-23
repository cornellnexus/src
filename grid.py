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
        # traversal_path: Ordered list of Node objects to travel to [Node list]

        # nodes_dict: Ordered dictionary of Nodes to travel to.
            - Keys are (y,x), aka (latitude, longitude), tuples.
            - Values are Node objects

        # obstacle_coordinates: List of Node objects that represent obstacle nodes

        lat_min: User input of minimum latitude boundary point
        lat_max: User input of maximum latitude boundary point
        long_min: User input of minimum longitude boundary point
        long_max: User input of maximum longitude boundary point
    """

    def __init__(self, lat_min, lat_max, long_min, long_max):
        self.traversal_path = []
        self.nodes_dict = {}
        self.obstacle_coordinates = []

        self.lat_min = lat_min
        self.lat_max = lat_max
        self.long_min = long_min
        self.long_max = long_max

        self.obstacle_length_limit = 10

        def calc_step(lat_min, lat_max, long_min, long_max):
            lat_range = geopy.distance.distance(
                (long_min, lat_min), (long_min, lat_max)).meters
            print(str(lat_range))
            long_range = geopy.distance.distance(
                (long_min, lat_min), (long_max, lat_min)).meters

            # Underestimate to achieve integer number of rows and columns
            # 1.8288 meters is 6 feet
            num_lat_steps = int(lat_range // 1.8288)
            print(str(num_lat_steps))
            num_long_steps = int(long_range // 1.8288)

            lat_step = (self.lat_max - self.lat_min) / num_lat_steps
            long_step = (self.long_max - self.long_min) / num_long_steps

            return lat_step, long_step, num_lat_steps+1, num_long_steps+1

        def generate_nodes(start_lat, start_long, row_num, col_num, lat_step, long_step):
            origin = (start_lat, start_long)
            traversal_path = []
            nodes_dict = {}

            true_min_lat = start_lat
            true_max_lat = start_lat + (row_num - 1) * lat_step

            for i in range(col_num):
                for j in range(row_num):
                    if i % 2 == 0:
                        node = Node(origin[0] + j*lat_step, origin[1] + i*long_step)
                        traversal_path.append(node)
                        nodes_dict[node.get_coords()] = node
                    elif i % 2 == 1:
                        lat_pos = origin[0] + \
                            ((row_num - 1) * lat_step) - j*lat_step
                        long_pos = origin[1] + i*long_step
                        node = Node(lat_pos, long_pos)
                        traversal_path.append(node)
                        nodes_dict[node.get_coords()] = node
            return traversal_path, nodes_dict, true_min_lat, true_max_lat

        self.lat_step, self.long_step, self.row_num, self.col_num = calc_step(lat_min, lat_max, long_min, long_max)
        self.traversal_path, self.nodes_dict, self.true_min_lat, self.true_max_lat = generate_nodes(
            lat_min, long_min, self.row_num, self.col_num, self.lat_step, self.long_step)


# TEST CASE
g = Grid(-76.488495, -76.488419, 42.444496, 42.444543)
print(g.traversal_path)

testlist = []
xlist = []
ylist = []

for node in g.traversal_path:
    coords = node.get_coords()
    xlist.append(coords[1])
    ylist.append(coords[0])

plt.plot(xlist, ylist, marker='o', markerfacecolor='blue')
plt.ylim(-76.488500,-76.488400)
plt.xlim(42.444444,42.444590)
plt.show()

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