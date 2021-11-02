from engine.node import Node
import numpy as np
from engine.kinematics import meters_to_lat, meters_to_long, get_vincenty_x, get_vincenty_y


class Grid:
    """
    Instances represent the current grid of the robot's traversal.

    INSTANCE ATTRIBUTES:
        # nodes: 2D Node List representing all the Node objects that make up the grid. [[Node List] List]
        # waypoints: Ordered list of Node objects to travel to that have not yet been traversed by the robot.

        # nodes_dict: Dictionary of all Node objects in the grid
            - Keys are (y,x), aka (latitude, longitude), tuples.
            - Values are Node objects

        # lat_min, lat_max, long_min, long_max: // doesn't make sense to have this be the actual map should just be of our grid
            minimum/maximum latitude/longitude boundary points of the actual map. [float]

        #num_rows: Number of rows of Nodes in the grid [int]
        #num_cols: Number of columns of Nodes in the grid. [int]
    """

    def __init__(self, lat_min, lat_max, long_min, long_max):
        STEP_SIZE_METERS = 2

        # ----------- HELPER FUNCTIONS FOR GRID INITIALIZATION ------------# 
        def calc_step(lat_min, lat_max, long_min, long_max, step_size_m):
            """
            Returns the number of rows and columns needed for a grid, given
            latitude and longitude boundaries and a desired step size between
            nodes in meters.

            Parameters:
            -----------
            # lat_min, lat_max, long_min, long_max: desired latitude and longitude boundaries of the grid [float]
            # step_size_m: step size in between nodes of the grid [float]
            """
            y_range = get_vincenty_y((lat_min, long_min), (lat_max, long_max))
            x_range = get_vincenty_x((lat_min, long_min), (lat_max, long_max))

            num_y_steps = int(y_range // step_size_m)
            num_x_steps = int(x_range // step_size_m)

            num_rows = num_y_steps + 1  # to account for starting node
            num_cols = num_x_steps + 1

            return num_rows, num_cols

        def generate_nodes(start_lat, start_long, rows, cols, step_size_m):
            """
            Returns a list of Node objects that make up the entire grid. [Node list]

            Parameters:
            -----------
            # start_lat: latitude coordinate of the robot's starting position [float]
            # start_long: longitude coordinate of the robot's starting position [float]
            # rows: # of rows in the grid [int]
            # cols: # of cols in the grid [int]
            # step_size_m: step size in between nodes of the grid (in meters) [float]
            """
            node_list = np.empty([rows, cols], dtype=np.object)

            gps_origin = (start_lat, start_long)

            lat_step = meters_to_lat(step_size_m)
            long_step = meters_to_long(step_size_m, start_lat)

            # Develop the gps grid and gps traversal path in order of lawnmower
            # traversal. 
            for i in range(cols):
                for j in range(rows):
                    is_border = (j == 0 or j == rows - 1)
                    if i % 2 == 0:
                        lat = gps_origin[0] + j * lat_step
                        long = gps_origin[1] + i * long_step
                        x = get_vincenty_x(gps_origin, (lat, long))
                        y = get_vincenty_y(gps_origin, (lat, long))
                        node = Node(lat, long, x, y, is_border)
                        node_list[j, i] = node
                    elif i % 2 == 1:
                        lat = gps_origin[0] + \
                                  ((rows - 1) * lat_step) - j * lat_step
                        long = gps_origin[1] + i * long_step
                        x = get_vincenty_x(gps_origin, (lat, long))
                        y = get_vincenty_y(gps_origin, (lat, long))
                        node = Node(lat, long, x, y, is_border)
                        row_index = rows - (j + 1)
                        node_list[row_index, i] = node

            return node_list

        # ----------------- GRID INITIALIZATION BEGINS ------------------- #
        self.lat_min = lat_min
        self.lat_max = lat_max
        self.long_min = long_min
        self.long_max = long_max

        self.num_rows, self.num_cols = calc_step(lat_min, lat_max, long_min, long_max, STEP_SIZE_METERS)

        self.nodes = generate_nodes(lat_min, long_min, self.num_rows, self.num_cols, STEP_SIZE_METERS)

    def get_num_rows(self):
        return self.num_rows

    def get_num_cols(self):
        return self.num_cols

    def get_waypoints(self, mode='lawn_full'):
        """
        Returns the robot's traversal path for the current grid. [Node list].

        Parameters:
        -----------
        # mode: The type of traversal path that is desired. [string]
            'lawn_full':
                - lawnmower traversal using every single node of the grid
                - starting node is the bottom left node of the grid

            'lawn_border'
                - lawnmower only the nodes in the top/bottom row of the grid
                - starting node is the bottom left node of the grid
        """
        waypoints = []
        node_list = self.nodes
        rows = node_list.shape[0]
        cols = node_list.shape[1]
        if mode == 'lawn_full':
            for i in range(cols):
                for j in range(rows):
                    if i % 2 == 0:
                        node = node_list[j, i]
                        waypoints.append(node)
                    elif i % 2 == 1:
                        row_index = rows - (j + 1)
                        node = node_list[row_index, i]
                        waypoints.append(node)
        # 'borders' mode only traverses the nodes in the top and bottom rows of the grid
        elif mode == 'lawn_border':
            for i in range(cols):
                if i % 2 == 0:
                    node1 = node_list[0, i]
                    node2 = node_list[rows - 1, i]
                    waypoints.append(node1)
                    waypoints.append(node2)
                elif i % 2 == 1:
                    node1 = node_list[rows - 1, i]
                    node2 = node_list[0, i]
                    waypoints.append(node1)
                    waypoints.append(node2)

        return waypoints
