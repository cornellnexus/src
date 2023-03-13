import math

import numpy

from engine.node import Node
import numpy as np
from engine.kinematics import meters_to_lat, meters_to_long, get_vincenty_x, get_vincenty_y
from enum import Enum


class Grid:
    class Direction(Enum):
        RIGHT = 1
        LEFT = 2
        UP = 3
        DOWN = 4

    class Orientation(Enum):
        """
        Clockwise or CounterClockwise
        """
        CW = 1
        CCW = 2
    """
    Instances represent the current grid of the robot's traversal.

    INSTANCE ATTRIBUTES:
        # lat_min, lat_max, long_min, long_max: minimum/maximum latitude/longitude boundary coordinates of the grid [float]

        # num_rows: Number of rows of Nodes in the grid [int]
        # num_cols: Number of columns of Nodes in the grid [int]

        # nodes: 2D Numpy array of Nodes representing all the nodes in the grid.
            Has dimensions [self.num_rows x self.num_cols]


        #num_rows: Number of rows of Nodes in the grid [int]
        #num_cols: Number of columns of Nodes in the grid. [int]

        #leftmost_node: the leftmost active node in the Grid which is used as the starting node in lawnmower and spiral traversal.
        #leftmost_node_pos: the (row,col) position of the leftmost node
        #border_nodes: all active nodes which either exist on the edge of the grid or have a neighbor that is an inactive node
        #active_waypoints_list: a list of active waypoints for every traversal algorithm. It is used to implement a color visualizaition of active waypoints.
        #inactive_waypoints_list: a list of inactive waypoints for every traversal algorithm. It is used to implement a color visualization of inactive waypoints.
     INSTANCE METHODS:
        # get_waypoints: Returns the ordered list of Node objects that the robot should travel to. This is
            determined by the desired type of traversal. [Node list]

    """

    def __init__(self, lat_min, lat_max, long_min, long_max):
        STEP_SIZE_METERS = 1

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
            Returns a 2D Numpy array of Node objects that make up the entire grid. Has dimensions [rows x cols]

            Parameters:
            -----------
            # start_lat: latitude coordinate of the robot's starting position [float]
            # start_long: longitude coordinate of the robot's starting position [float]
            # rows: # of rows in the grid [int]
            # cols: # of cols in the grid [int]
            # step_size_m: step size in between nodes of the grid (in meters) [float]
            """
            node_list = np.empty([rows, cols], dtype=object)

            gps_origin = (start_lat, start_long)

            lat_step = meters_to_lat(step_size_m)
            long_step = meters_to_long(step_size_m, start_lat)

            # Develop the gps grid and gps traversal path in order of lawnmower
            # traversal.
            for i in range(cols):
                for j in range(rows):
                    if i % 2 == 0:
                        lat = gps_origin[0] + j * lat_step
                        long = gps_origin[1] + i * long_step
                        x = get_vincenty_x(gps_origin, (lat, long))
                        y = get_vincenty_y(gps_origin, (lat, long))
                        node = Node(lat, long, x, y)
                        node_list[j, i] = node
                    elif i % 2 == 1:
                        lat = gps_origin[0] + \
                            ((rows - 1) * lat_step) - j * lat_step
                        long = gps_origin[1] + i * long_step
                        x = get_vincenty_x(gps_origin, (lat, long))
                        y = get_vincenty_y(gps_origin, (lat, long))
                        node = Node(lat, long, x, y)
                        row_index = rows - (j + 1)
                        node_list[row_index, i] = node

            return node_list

        # ----------------- GRID INITIALIZATION BEGINS ------------------- #
        self.lat_min = lat_min
        self.lat_max = lat_max
        self.long_min = long_min
        self.long_max = long_max

        self.num_rows, self.num_cols = calc_step(
            lat_min, lat_max, long_min, long_max, STEP_SIZE_METERS)

        self.nodes = generate_nodes(
            lat_min, long_min, self.num_rows, self.num_cols, STEP_SIZE_METERS)
        self.border_nodes = None
        self.leftmost_node = None
        self.leftmost_node_pos = None
        self.active_waypoints_list = []
        self.inactive_waypoints_list = []

        # Properties used to determine our adjustable grid traversal pathing
        self.waypoints_is_finished = False
        self.curr_pos = None
        self.direction = self.Direction.RIGHT
        self.waypoints = []

    def get_active_waypoints_list(self):
        return self.active_waypoints_list

    def get_inactive_waypoints_list(self):
        return self.inactive_waypoints_list

    def get_num_rows(self):
        return self.num_rows

    def get_num_cols(self):
        return self.num_cols

    # --------------------- METHODS TO ACTIVATE NODES ON THE GRID -------------- #

    def activate_node(self, row, col):
        """
        Activates the node at the given location.
        """
        self.nodes[row][col].activate_node()

    def activate_nodes(self, row, col, row_limit, col_limit):
        """
        Activates all the nodes in the given range.
        """
        for y in range(row, row_limit):
            for x in range(col, col_limit):
                self.activate_node(y, x)

    def determine_active_waypoints(self, node):
        """
        Determines whether a waypoint on a traversal algorithm is active. If active, the node is
        appended to active_waypoints. Else, if inactive, the node is appended to inactive_waypoints.
        """
        if node.is_active:
            self.active_waypoints_list.append(node)
        else:
            self.inactive_waypoints_list.append(node)

    # --------------------- METHODS TO FINISH INITIALIZATION OF ACTIVATED GRID -------------- #

    def is_on_border(self, row, col, row_limit, col_limit):
        """
        Returns whether a particular activated node is on the border.

        An activated node is on the border if any of the following conditions hold:
        1. Any of its neighboring nodes are inactive.
        2. It exists on the very edge of the grid.
        """
        min_col = max(0, col - 1)
        min_row = max(0, row - 1)
        max_col = min(col_limit, col + 1)
        max_row = min(row_limit, row + 1)

        # If this node is on the very edge of the grid, it is automatically a border node
        if min_col == 0 or min_row == 0 or max_col == col_limit or max_row == row_limit:
            return True

        # If the node has a neighboring node that is inactive, it is a border node
        for col in range(min_col, max_col):
            for row in range(min_row, max_row):
                if not self.nodes[row][col].is_active_node():
                    return True
        return False

    # --------------------- ADJUSTABLE TRAVERSAL ALGORITHMS -------------- #

    def get_active_neighbor_node(self, row, col, row_max, col_max):
        """
        Returns the active neighbor node at the given row and col position.

        If the row/col position is out of bounds or no active node exists at the
        given location, None is returned.
        """
        if row < 0 or row >= row_max or col < 0 or col >= col_max:
            return None

        node_list = self.nodes
        neighbor_node = node_list[row][col]
        if not neighbor_node.is_active_node():
            return None
        else:
            return neighbor_node

    def activate_rectangle(self, row, col, row_limit, col_limit):
        """
        Activates all the nodes in a rectangle based on row start/end, col
        start/end.
        """
        for x in range(row, row_limit):
            for y in range(col, col_limit):
                self.nodes[x, y].is_active = True
        return self.nodes

    def activate_circle(self, circle_center_row, circle_center_col, circle_radius):
        """
        Activates all the nodes in a circle based on center and radius.
        """
        for x in range(self.num_rows):
            for y in range(self.num_cols):
                if ((x - circle_center_row) ** 2 + (y - circle_center_col) ** 2 - circle_radius ** 2) < 0:
                    self.nodes[x, y].is_active = True

    def isInsideTriangle(self, p1, p2, p3, p4):
        """ A function to check whether point P(x, y) lies inside the triangle formed by
        A(x1, y1), B(x2, y2) and C(x3, y3)"""
        def area(p1, p2, p3):
            return abs((p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1])
                        + p3[0] * (p1[1] - p2[1])) / 2.0)

        # Calculate area of triangle ABC
        A = area(p1, p2, p3)

        # Calculate area of triangle PBC
        A1 = area(p4, p2, p3)

        # Calculate area of triangle PAC
        A2 = area(p1, p4, p3)

        # Calculate area of triangle PAB
        A3 = area(p1, p2, p4)

        # Check if sum of A1, A2 and A3
        # is same as A
        return A == (A1 + A2 + A3)

    def activate_line(self, row, col, n, isHorizontal):
        """
        Activate a single horizontal line of length n starting at row, col.
        If isHorizontal is True, the line is from (row, col) to (row, col + n).
        Otherwise, the line is from (row, col) to (row + n, col).
        """
        # Note: rows are y-position and columns are x-position
        if isHorizontal:
            for i in range(n):
                self.nodes[col+i][row].is_active = True
        else:
            for i in range(n):
                self.nodes[col][row+i].is_active = True

    def activate_triangle(self, p1, p2, p3):
        """
        Activates all the nodes in a triangle based on three vertices p1, p2, p3.
        """
        for x in range(self.num_rows):
            for y in range(self.num_cols):
                if self.isInsideTriangle(p1, p2, p3, (x, y)):
                    self.nodes[x, y].is_active = True

    def is_on_border(self, row, col, row_limit, col_limit):
        """
        Checks if node is on border of activated nodes
        """
        min_col = max(0, col - 1)
        min_row = max(0, row - 1)
        max_col = min(col_limit, col + 1)
        max_row = min(row_limit, row + 1)

        # If this node is on the very edge of the grid, it is automatically a border node
        if row == 0 or col == 0 or col == self.get_num_cols() - 1 or row == self.get_num_rows() - 1:
            return True

        # If the node has a neighboring node that is inactive, it is a border node
        for col in range(min_col, max_col + 1):
            for row in range(min_row, max_row + 1):
                if not self.nodes[row][col].is_active:
                    return True
        return False

    def find_border_nodes(self):
        """
            Find all activated border nodes on the grid.
            Based on these activated notes, finds the border nodes.

            This function loops through all the nodes, checks if a particular node has
            been activated, and if so checks to see if that node is a border node. At the
            end of the function call, fields 'leftmost_node', 'leftmost_node_pos', and
            'border_nodes' will be initialized.
        """
        if self.direction == self.Direction.LEFT or self.direction == self.Direction.RIGHT:
            border_list = []
            leftmost_node = None
            leftmost_node_pos = None

            for row in range(self.num_rows):
                for col in range(self.num_cols):
                    node = self.nodes[row][col]
                    if node.is_active and self.is_on_border(row, col, self.num_rows, self.num_cols):
                        # check if this is an active node and on the border
                        self.nodes[row][col].is_border = True
                        border_list.append((node, row, col))
                        if leftmost_node_pos is None or col < leftmost_node_pos[1]:
                            leftmost_node = node
                            leftmost_node_pos = (row, col)

            self.border_nodes = border_list
            self.leftmost_node = leftmost_node
            self.leftmost_node_pos = leftmost_node_pos
        else:
            border_list = []
            leftmost_node = None
            leftmost_node_pos = None

            for row in range(self.num_rows):
                for col in range(self.num_cols):
                    node = self.nodes[row][col]
                    if node.is_active and self.is_on_border(row, col, self.num_rows, self.num_cols):
                        # check if this is an active node and on the border
                        self.nodes[row][col].is_border = True
                        border_list.append((node, row, col))
                        if leftmost_node_pos is None or row < leftmost_node_pos[0]:
                            leftmost_node = node
                            leftmost_node_pos = (row, col)

            self.border_nodes = border_list
            self.leftmost_node = leftmost_node
            self.leftmost_node_pos = leftmost_node_pos

 # --------------------- HELPER FUNCTIONS FOR TRAVERSAL CREATION -------------- #
    def edge_column_of_next_row(self, pos):
        """
        If the direction we are heading is Left, returns the column of the innermost
        (closer to the center of the shape) leftmost node that is
            1) active
            2) not border node
        between the current row in [pos] and the next row above [pos].

        If the direction we are heading is Right, returns the column of the innermost
        (closer to the center of the shape) rightmost node that is
            1) active
            2) not border node
        between the current row in [pos] and the next row above [pos].

        If the direction we are heading is Up, returns the row of the innermost
        (closer to the center of the shape) top node that is
            1) active
            2) not border node
        between the current col in [pos] and the next col above [pos].

        If the direction we are heading is Down, returns the row of the innermost
        (closer to the center of the shape) bottom node that is
            1) active
            2) not border node
        between the current col in [pos] and the next col above [pos].

        If there are no more nodes in the next row, return None.

        pos: the current position (col,row)
        """
        # Obtain all border nodes in the next and current rows
        if self.direction == self.Direction.RIGHT or self.direction == self.Direction.LEFT:
            # Note: node_info: (node, row, col)

            # Obtain all border_nodes on the next col
            candidate_nodes_next = [
                node_info for node_info in self.border_nodes if node_info[2] == pos[1] + 1]
            # Obtain all border_nodes on the current col
            candidate_nodes_curr = [
                node_info for node_info in self.border_nodes if node_info[2] == pos[1]]
        else:  # the direction is UP or DOWN
            # Obtain all border_nodes on the next row
            candidate_nodes_next = [
                node_info for node_info in self.border_nodes if node_info[1] == pos[0] + 1]
            # all border_nodes on current row
            candidate_nodes_curr = [
                node_info for node_info in self.border_nodes if node_info[1] == pos[0]]

        if (candidate_nodes_next == [] and candidate_nodes_curr == []):
            # There are no border nodes in the current row or next row
            return None

        # node_info_curr is the last node we would traverse on the current row
        if self.direction == self.Direction.RIGHT:
            node_info_curr = max(candidate_nodes_curr,
                                 key=lambda node_info: node_info[1])
        elif self.direction == self.Direction.LEFT:
            node_info_curr = min(candidate_nodes_curr,
                                 key=lambda node_info: node_info[1])
        elif self.direction == self.Direction.UP:
            node_info_curr = max(candidate_nodes_curr,
                                 key=lambda node_info: node_info[2])
        elif self.direction == self.Direction.DOWN:
            node_info_curr = min(candidate_nodes_curr,
                                 key=lambda node_info: node_info[2])

        if (candidate_nodes_next == []):
            # Since there we have reached the last row that we can traverse,
            # let's traverse this last row
            if self.direction == self.Direction.RIGHT or self.direction == self.Direction.LEFT:
                # return the x-axis of the last node we will traverse in the grid
                return node_info_curr[1]
            else:
                return node_info_curr[2]
        else:
            if self.direction == self.Direction.RIGHT:
                # compare it with the last node in the next row so we don't go overbounds during turning
                node_info_next = max(candidate_nodes_next,
                                     key=lambda node_info: node_info[1])
                return min(node_info_next[1], node_info_curr[1])
            elif self.direction == self.Direction.LEFT:
                node_info_next = min(candidate_nodes_next,
                                     key=lambda node_info: node_info[1])
                return max(node_info_next[1], node_info_curr[1])
            elif self.direction == self.Direction.UP:
                node_info_next = max(candidate_nodes_next,
                                     key=lambda node_info: node_info[2])
                return min(node_info_next[2], node_info_curr[2])

            elif self.direction == self.Direction.DOWN:
                node_info_next = min(candidate_nodes_next,
                                     key=lambda node_info: node_info[2])
                return max(node_info_next[2], node_info_curr[2])

    def plot_circle(self, start_pos, end_pos, center, orientation, theta_step=math.pi / 12):
        """
        Returns a circle of nodes starting from the [start_pos], going at orientation [orientation], and ending at the [end_pos] with center [center].
        Arguments:
            start_pos: float tuple representing the starting point of the circle being plotted
            end_pos: float tuple representing the ending point of the circle being plotted
            center: float tuple representing the center point of the circle being plotted
            orientation: orientation that the nodes are being plotted from, starting with the start_pos and ending at the end_pos
            theta_step: float representing the angle step when plotting the turning arch. The
            smaller the value, the smoother the curve will be. Default value = math.pi/12.
        """
        r = math.hypot(float(start_pos[0]) - center[0],
                       float(start_pos[1]) - center[1])

        theta_init = math.atan2(
            start_pos[1] - center[1], start_pos[0] - center[0])
        theta_end = math.atan2(end_pos[1] - center[1], end_pos[0] - center[0])
        circle_plt = []
        if orientation == self.Orientation.CCW:
            if theta_end < theta_init:
                theta_end = theta_end + 2 * math.pi
            theta = theta_init
            while theta < theta_end:
                circle_plt.append(
                    (r * math.cos(theta) + center[0], r * math.sin(theta) + center[1]))
                theta = theta + theta_step
        elif orientation == self.Orientation.CW:
            if theta_init < theta_end:
                theta_init = theta_init + 2 * math.pi
            theta = theta_init
            while theta > theta_end:
                circle_plt.append(
                    (r * math.cos(theta) + center[0], r * math.sin(theta) + center[1]))
                theta = theta - theta_step
        else:
            raise Exception("invalid orientation")
        return circle_plt

    def get_turning_column(self, edge_column):
        """
        Returns the column where the robot should begin turning in a guided traversal.

        Arguments:
            edge_column: int representing the column of the
            leftmost/rightmost node in the row above the current row being
            traversed.
        """
        if self.direction == self.Direction.LEFT or self.direction == self.Direction.DOWN:
            return edge_column + 1
        elif self.direction == self.Direction.RIGHT or self.direction == self.Direction.UP:
            return edge_column - 1

    def is_before_turning_column(self, turning_column):
        """
        Returns whether the current waypoint creation position is still before
        the column where it should turn.

        Arguments:
            turning_column: int representing the column of where it should turn.
        """
        if self.direction == self.Direction.LEFT:
            return self.curr_pos[0] <= (turning_column)
        elif self.direction == self.Direction.RIGHT:
            return self.curr_pos[0] >= (turning_column)
        elif self.direction == self.Direction.UP:
            return self.curr_pos[1] >= (turning_column)
        elif self.direction == self.Direction.DOWN:
            return self.curr_pos[1] <= (turning_column)

    def switch_directions(self):
        """
        Switch the current waypoint creation direction.
        """
        if self.direction == self.Direction.LEFT:
            self.direction = self.Direction.RIGHT
        elif self.direction == self.Direction.RIGHT:
            self.direction = self.Direction.LEFT
        elif self.direction == self.Direction.UP:
            self.direction = self.Direction.DOWN
        elif self.direction == self.Direction.DOWN:
            self.direction = self.Direction.UP

    def get_next_traversal_pos(self):
        """
        Returns the next position along the row with resepect to the current
        waypoint creation position.
        """
        if self.direction == self.Direction.LEFT:
            return (self.curr_pos[0] - 1, self.curr_pos[1])
        elif self.direction == self.Direction.RIGHT:
            return (self.curr_pos[0] + 1, self.curr_pos[1])
        elif self.direction == self.Direction.UP:
            return (self.curr_pos[0], self.curr_pos[1] + 1)
        elif self.direction == self.Direction.DOWN:
            return (self.curr_pos[0], self.curr_pos[1] - 1)

    def get_turn_orientation(self):
        """
        Returns the turning orientation between rows depending on the
        incoming traversal direction.
        """
        if self.direction == self.Direction.LEFT or self.direction == self.Direction.UP:
            return self.Orientation.CW
        elif self.direction == self.Direction.RIGHT or self.direction == self.Direction.DOWN:
            return self.Orientation.CCW

    def add_guided_waypoints(self, is_vertical):
        """
        Generates waypoints for a guided lawnmower traversal.

        For this traversal, the robot should traverse row by row until there are
        no more active nodes that are encaptured by the border nodes.

        When the robot reaches a the first node in a row of active nodes, it
        will traverse down the row and turn to the next row BEFORE the edge node,
        the last node in the row (so the turn is contained within the bounds of
        the selected border).

        Ex:  3 <- 4 <- 5
                    ^
             0 -> 1 -> 2
        """
        # The robot is traversing in the left direction, from right to left
        # The next position in the left direction
        new_pos = self.get_next_traversal_pos()

        # The next leftmost position in row above
        edge_column = self.edge_column_of_next_row(self.curr_pos)

        if edge_column == None:  # Problem with initialization
            self.waypoints_is_finished = True
            return

        # Column where we want to begin turning, one node closer to the inside of our shape
        turning_column = self.get_turning_column(edge_column)
        if is_vertical:
            next_row = self.curr_pos[0] + 1
        else:
            next_row = self.curr_pos[1] + 1

        # Traverse the row until it is time to turn
        while not self.is_before_turning_column(turning_column):
            self.waypoints.append(new_pos)
            self.curr_pos = new_pos
            new_pos = self.get_next_traversal_pos()

        # Check whether turning to the next row is valid
        if (is_vertical and not self.nodes[next_row][turning_column].is_active)\
                or (not is_vertical and not (next_row < self.num_cols and self.nodes[turning_column][next_row].is_active)):
            # Next row is out of bounds of the grid OR the node in the row/column above,
            # where we are trying to turn to, isn't active for traversal
            self.waypoints_is_finished = True

        else:
            # Turn to next row
            # Create waypoints needed for a smooth turning trajectory to
            # "guide" our robot to the next original waypoint
            if is_vertical:
                circle_plt = self.plot_circle((self.curr_pos[0], turning_column), (next_row, turning_column),
                                              (self.curr_pos[0] + .5, turning_column), self.get_turn_orientation())
            else:
                circle_plt = self.plot_circle((turning_column, self.curr_pos[1]), (turning_column, next_row),
                                              (turning_column, self.curr_pos[1] + .5), self.get_turn_orientation())
            self.waypoints += circle_plt
            # Update traversal details
            self.switch_directions()
            if is_vertical:
                self.curr_pos = (next_row, turning_column)
            else:
                self.curr_pos = (turning_column, next_row)

    def get_all_guided_lawnmower_waypoints_adjustable(self, is_vertical):
        """
        Returns the waypoints being traversed. The algorithm traverses from bottom
        up starting from left to right.

        There are two big branches:
            * Direction.LEFT branch handles traversal from right towards left and
            turning clock wise at the end of the row (or terminating).
            * Direction.RIGHT branch handles traversal from left towards right
            and turning counter clock wise at the end of the row (or terminating).
        The plot_circle plots the circular waypoints during the turn
        """
        # TODO: move find border nodes into this func instead of doing it
        # everytime before this is called
        self.curr_pos = self.leftmost_node_pos
        self.waypoints.append(self.curr_pos)
        # TODO: Change to allow choosing down and left directions
        if is_vertical:
            self.direction = self.direction.UP
        while not self.waypoints_is_finished:
            if self.curr_pos == None:
                self.waypoints_is_finished = True

            # Handle traversal from left/right and turning CW/CCW at the end of
            # the row (or terminating).

            self.add_guided_waypoints(is_vertical)

        return self.waypoints

    # --------------------- STANDARD TRAVERSAL ALGORITHMS -------------- #

    def get_spiral_waypoints(self):
        """
        Returns the robot's spiral traversal path for the current grid using every
        single node of the grid. [Node list].
        """
        waypoints = []
        node_list = self.nodes

        col = 0  # start at bottom left corner
        row = 0

        # these tuples simulate the robot's next movement based on turn state
        step_col = (1, 0, -1, 0)
        step_row = (0, 1, 0, -1)
        turn_state = 0  # turn_state is a variable that must be between 0..3

        for _ in range(self.num_rows * self.num_cols):  # for loop over all nodes
            node = node_list[row, col]
            self.determine_active_waypoints(node)
            waypoints.append(node)
            next_col = col + step_col[turn_state]
            next_row = row + step_row[turn_state]
            if 0 <= next_col < self.num_cols and 0 <= next_row < self.num_rows and not node_list[
                    next_row, next_col] in waypoints:
                col = next_col
                row = next_row
            else:
                turn_state = (turn_state + 1) % 4
                next_col = col + step_col[turn_state]
                next_row = row + step_row[turn_state]
                col = next_col
                row = next_row
        waypoints.reverse()
        return waypoints

    def get_all_lawnmower_waypoints(self):
        """
        Returns the robot's lawnmower traversal path for the current grid using every
        single node of the grid. Starting node is the bottom left node of the list. [Node list].
        """
        waypoints = []
        node_list = self.nodes
        for i in range(self.num_cols):
            for j in range(self.num_rows):
                is_border = (j == 0 or j == self.num_rows - 1) # leftmost and rightmost nodes are considered border nodes
                if i % 2 == 0:
                    node = node_list[j, i]
                    node.is_border = is_border
                    self.determine_active_waypoints(node)
                    waypoints.append(node)
                elif i % 2 == 1:
                    # adds the nodes in reverse order (not zigzag)
                    row_index = self.num_rows - (j + 1)
                    node = node_list[row_index, i]
                    node.is_border = is_border
                    self.determine_active_waypoints(node)
                    waypoints.append(node)
        return waypoints

    def get_border_lawnmower_waypoints(self):
        """
        Returns the robot's lawnmower border traversal path for the current grid using
        only nodes in the top/bottom row of the grid. Starting node is the bottom left
        node of the list. [Node list].
        """
        waypoints = []
        node_list = self.nodes
        for i in range(self.num_cols):
            if i % 2 == 0:
                node1 = node_list[0, i]
                node2 = node_list[self.num_rows - 1, i]
                self.determine_active_waypoints(node1)
                self.determine_active_waypoints(node2)
                waypoints.append(node1)
                waypoints.append(node2)
            elif i % 2 == 1:
                node1 = node_list[self.num_rows - 1, i]
                node2 = node_list[0, i]
                self.determine_active_waypoints(node1)
                self.determine_active_waypoints(node2)
                waypoints.append(node1)
                waypoints.append(node2)
        return waypoints

    def get_straight_line_waypoints(self, y_start_row=0, y_start_pct=None):
        """
        Returns the robot's lawnmower border traversal path for the current grid using
        only nodes in a straight line . Starting node is the left-most node starting at
        the row in [Node List] that corresponds to how high the straight line should start at.

        PARAMETERS:
        ---------
        y_start_row: What row height that we want to start the straight line.
        y_start_pct: What percentage height that we want to start the straight line.(ex: 0.5: if there are 20 rows,
                     start straight line at the 10th row up) Default:None
        """
        waypoints = []
        node_list = self.nodes
        if y_start_pct is not None:
            selected_row = int(y_start_pct * self.num_rows)
        else:
            selected_row = y_start_row
        for i in range(self.num_cols):
            selected_row_node = node_list[selected_row][i]
            self.determine_active_waypoints(selected_row_node)
            waypoints.append(selected_row_node)
        return waypoints

    def get_waypoints(self, mode):
        """
        Returns the robot's traversal path for the current grid. [Node list].

        Returns empty list if [mode] is not one of [ControlMode.LAWNMOWER],
        [ControlMODE.LAWNMOWER_B], or [ControlMODE.SPIRAL].

        Parameters:
        -----------
        # mode: The type of traversal path that is desired. [ControlMode].
            LAWNMOWER:
                - lawnmower traversal using every single node of the grid
                - starting node is the bottom left node of the grid

            LAWNMOWER_B:
                - lawnmower traversal using only the nodes in the top/bottom row of the grid
                - starting node is the bottom left node of the grid

            SPIRAL:
                -spiral traversal using every single node of the grid
                -starting node varies based on the width/height of the grid
                -ending node is the bottom left node of the grid
            STRAIGHT:
                - straight_line traversal across a row of nodes
                - starting node is left most node at selected row
                - ending node right most node at selected row
        """
        from engine.mission import ControlMode  # import placed here to avoid circular import
        if mode == ControlMode.LAWNMOWER:
            waypoints = self.get_all_lawnmower_waypoints()
        elif mode == ControlMode.LAWNMOWER_B:
            waypoints = self.get_border_lawnmower_waypoints()
        elif mode == ControlMode.SPIRAL:
            waypoints = self.get_spiral_waypoints()
        elif mode == ControlMode.STRAIGHT:
            waypoints = self.get_straight_line_waypoints(y_start_pct=0.5)
        elif mode == ControlMode.LAWNMOWER_GUIDED:
            waypoints = self.get_all_guided_lawnmower_waypoints_adjustable(
                True)
        else:
            return []
        return waypoints
