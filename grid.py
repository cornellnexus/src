from node import Node
import numpy as np
import math
import serial
import geopy.distance
import matplotlib.pyplot as pltß
from math import log
import time
from haversine import haversine, Unit


class Grid:
    """
    Instances represent the current grid of the robot's traversal.

    INSTANCE ATTRIBUTES:
        # traversal_path: Ordered list of Node objects that have not yet been 
                          traversed by the robot. [Node list]

        # nodes_dict: Dictionary of all Node objects in the grid
            - Keys are (y,x), aka (latitude, longitude), tuples.
            - Values are Node objects

        # obstacle_coordinates: List of Node objects that represent obstacle nodes [Node list]

        #lat_min: Minimum latitude boundary point of the actual map. [float]
        #lat_max: Maximum latitude boundary point of the actual map. [float]
        #long_min: Minimum longitude boundary point of the actual map. [float]
        #long_max: Maximum longitude boundary point of the actual map. [float]

        #lat_step: Latitude step size in between Nodes of the grid
        #long_step: Longitude step size in between Nodes of the grid 
        #row_num: Number of rows of Nodes in the grid
        #col_num: Number of columns of Nodes in the grid.
        #true_min_lat: Minimum latitude boundary point of the generated grid. 
        #true_max_lat: Maximum latitude boundary point of the generated grid.
        #true_min_long: Minimum longitude boundary point of the generated grid. 
        #true_max_long: Maximum longitude boundary point of the generated grid.
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
            """
            Given latitude and longitude bounds for the desired space of the robot's 
            traversal, calculates the step size in between nodes in longitude and 
            latitude units using the desired size in meters. 
            (Desired space btw nodes is currently 6 feet ~= 1.8288 m)
            
            Returns: latitude & longitude step size, number of rows, and number 
            of columns.
            """
            lat_range = haversine(
                (long_min, lat_min), (long_min, lat_max), unit = Unit.METERS)
            long_range = haversine(
                (long_min, lat_min), (long_max, lat_min), unit = Unit.METERS)

            desired_step_size = 1.8288
            # Underestimate using int() to achieve integer number of rows and columns
            num_lat_steps = int(lat_range // desired_step_size)
            num_long_steps = int(long_range // desired_step_size)

            lat_step = (self.lat_max - self.lat_min) / num_lat_steps
            long_step = (self.long_max - self.long_min) / num_long_steps

            num_rows = num_lat_steps+1
            num_cols = num_long_steps+1

            return lat_step, long_step, num_rows, num_cols

        def generate_nodes(start_lat, start_long, row_num, col_num, lat_step, long_step):
            """
            Given the starting position of the robot, the number of rows/cols of Nodes 
            and the step size in between the Nodes, creates and returns a
            dictionary and ordered list of all the Node objects in the grid.
            Also, returns the longitude and latitude boundaries of the generated grid. 
            """
            origin = (start_lat, start_long)
            traversal_path = []
            nodes_dict = {}

            true_min_long = start_long
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
            return traversal_path, nodes_dict, true_min_lat, true_max_lat, true_min_long

        self.lat_step, self.long_step, self.row_num, self.col_num = calc_step(lat_min, lat_max, long_min, long_max)
        self.traversal_path, self.nodes_dict, self.true_min_lat, self.true_max_lat, self.true_min_long = generate_nodes(
            lat_min, long_min, self.row_num, self.col_num, self.lat_step, self.long_step)

# g = Grid(-76.4,-76.2,42.0,42.4)
# print(g.traversal_path)

# g = Grid(0,0.01,0,0.01)
# g = Grid(42.4596, 42.4642, -76.5119, -76.5013)

# print(g.traversal_path)
