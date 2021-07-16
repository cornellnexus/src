from node import Node
import numpy as np
import serial
import geopy.distance
import matplotlib.pyplot as pltß
from haversine import haversine, Unit
from vincenty import vincenty
from kinematics import meters_to_lat,meters_to_long,get_vincenty_x,get_vincenty_y,get_haversine_x,get_haversine_y


class Grid():
    """
    Instances represent the current grid of the robot's traversal.

    INSTANCE ATTRIBUTES:
        # gps_grid
        # gps_waypoints: Ordered list of Node objects that have not yet been 
                          traversed by the robot. [Node list]
        # meters_grid

        # nodes_dict: Dictionary of all Node objects in the grid
            - Keys are (y,x), aka (latitude, longitude), tuples.
            - Values are Node objects

        #lat_min: Minimum latitude boundary point of the actual map. [float]
        #lat_max: Maximum latitude boundary point of the actual map. [float]
        #long_min: Minimum longitude boundary point of the actual map. [float]
        #long_max: Maximum longitude boundary point of the actual map. [float]

        #num_rows: Number of rows of Nodes in the grid [int]
        #num_cols: Number of columns of Nodes in the grid. [int]
    """

    def __init__(self, lat_min, lat_max, long_min, long_max):
        STEP_SIZE_METERS = 2

        # ----------- HELPER FUNCTIONS FOR GRID INITIALIZATION ------------# 
        def calc_step(lat_min, lat_max, long_min, long_max,step_size_m):
            """
            Calculates the number of rows and columns needed for a grid with given 
            latitude and longitude boundaries and a desired step size in meters. 
            """
            y_range = get_vincenty_y((lat_min,long_min), (lat_max,long_max))
            x_range = get_vincenty_x((lat_min,long_min), (lat_max,long_max))

            num_y_steps = int(y_range // step_size_m)
            num_x_steps = int(x_range // step_size_m)

            num_rows = num_y_steps+1 # to account for starting node
            num_cols = num_x_steps+1

            return num_rows, num_cols

        def generate_nodes(start_lat, start_long, rows, cols, step_size_m):
            """
            Returns a grid of GPS Node objects [Node np array] and the GPS grid's
            corresponding traversal path [Node list]. 
            """
            gps_grid = np.ndarray([rows,cols],dtype=np.object)
            origin = (start_lat, start_long)
            gps_traversal_path = []

            lat_step = meters_to_lat(step_size_m)
            long_step = meters_to_long(step_size_m, start_lat)
            
            true_max_lat = start_lat + (rows - 1) * lat_step
            true_max_long = start_long + (cols - 1) * long_step

            # Develop the gps grid and gps traversal path in order of lawnmower
            # traversal. 
            for i in range(cols):
                for j in range(rows):
                    is_border = (j == 0 or j == rows-1)
                    if i % 2 == 0:
                        node = Node(origin[0] + j*lat_step, origin[1] + i*long_step,is_border)
                        gps_traversal_path.append(node)
                        gps_grid[j,i] = node
                    elif i % 2 == 1:
                        lat_pos = origin[0] + \
                            ((rows - 1) * lat_step) - j*lat_step
                        long_pos = origin[1] + i*long_step
                        node = Node(lat_pos, long_pos,is_border)
                        gps_traversal_path.append(node)
                        row_index = rows - (j+1)
                        gps_grid[row_index,i] = node
                             
            return gps_grid, gps_traversal_path, true_max_lat, true_max_long

        def grid_convert(gps_grid):
            """
            Returns a grid [Node np array] in units of meters that corresponds 
            to the gps_grid input. 
            """
            meters_grid = np.ndarray(gps_grid.shape,dtype = np.object)
            rows = gps_grid.shape[0]
            cols = gps_grid.shape[1]
            gps_origin = (gps_grid[0,0]).get_coords()
            for i in range(cols):
                for j in range(rows):
                    is_border = (j == 0 or j == rows-1)
                    curr_coords = (gps_grid[j,i]).get_coords()
                    x_dist = get_vincenty_x(gps_origin,curr_coords)
                    y_dist = get_vincenty_y(gps_origin,curr_coords)
                    meters_grid[j,i] = Node(x_dist,y_dist,is_border)
                    
            return meters_grid

        # ----------------- GRID INITIALIZATION BEGINS ------------------- #
        self.lat_min = lat_min
        self.lat_max = lat_max
        self.long_min = long_min
        self.long_max = long_max

        self.obstacle_length_limit = 10
        self.num_rows, self.num_cols = calc_step(lat_min, lat_max, long_min, long_max,STEP_SIZE_METERS)
        self.gps_grid, self.gps_waypoints, self.true_lat_max, self.true_long_max = generate_nodes(
            lat_min, long_min, self.num_rows, self.num_cols, STEP_SIZE_METERS)
        self.meters_grid = grid_convert(self.gps_grid)

    def get_num_rows(self):
        return self.num_rows 
    
    def get_num_cols(self):
        return self.num_cols
        
    def get_waypoints(self,mode='full'):
        """
        Returns the GPS traversal path in terms of meters for the current grid. 
        """
        waypoints = []
        meters_grid = self.meters_grid
        rows = meters_grid.shape[0]
        cols = meters_grid.shape[1]
        if mode == 'full':
            for i in range(cols):
                for j in range(rows):
                    if i % 2 == 0:
                        node = meters_grid[j,i]
                        waypoints.append(node)
                    elif i % 2 == 1:
                        row_index = rows - (j+1)
                        node = meters_grid[row_index,i]
                        waypoints.append(node)
        # 'borders' mode only traverses the nodes in the top and bottom rows of the grid
        elif mode == 'borders': 
            for i in range(cols):
                if i % 2 == 0:
                    node1 = meters_grid[0,i]
                    node2 = meters_grid[rows-1,i]
                    waypoints.append(node1)
                    waypoints.append(node2)
                elif i % 2 == 1:
                    node1 = meters_grid[rows-1,i]
                    node2 = meters_grid[0,i]
                    waypoints.append(node1)
                    waypoints.append(node2)

        return waypoints
