from coordinate import Coordinate
import time
import numpy as np
import csv
import math
import numpy
import serial
from Vector import Vector


class Obstacle:
    """
    Instances represents the list of nodes that make up the robot's traversal path along the graph.

    INSTANCE ATTRIBUTES:
        traversalPath: Ordered list of Coordinate objects to travel to [Coordinate list]

        traversalDict: Ordered dictionary of nodes to travel to where keys are (x,y) tuples and values are Coordinate objects

        obstacle_coordinates: List of Coordinate objects that represent obstacle nodes

        lat_min: User input of minimum latitude boundary point
        lat_max: User input of maximum latitude boundary point
        long_min: User input of minimum longitude boundary point
        long_max: User input of maximum longitude boundary point
    """

    traversalPath = []
    traversalDict = {}
    obstacle_coordinates = []
    # 42.445084, -76.482847
    # 42.445086, -76.482364
    #

    # Declaration of latitude min/max, and longitude min/max. These will be assigned
    # by user input
    lat_min = 0
    lat_max = 0
    long_min = 0
    long_max = 0
    # Declaration of longitude step and latitude step. These are determined based on
    # the differences between longitude max/min and latitude max/min, respectuflly.
    # 2 times the width of the robot (future implementations)
    long_step = 0
    lat_step = 0

    # ser = serial.Serial('/dev/cu.usbserial-1420', 9600)
    ser = serial.Serial("/dev/cu.usbserial-1420", 9600, timeout=1)

    # Time to wait before checking for distance data again.
    distance_check_wait_time = 5

    # in the future, probably location of base station
    robot_starting_position = (lat_min, long_min)

    # Initialization of variables
    # User inputs longitude min/max and latitude min/max for Initialization
    # Based on user inputs, the long step and latitude step are created
    # With the user inputs and steps, the coordinates are created
    def __init__(self, longMin, longMax, latMin, latMax):
        # self.xMax = longMax #Graph boundary on x-axis
        # self.yMax = latMax  #Graph boundary on y-axis

        self.long_min = longMin
        self.long_max = longMax
        self.lat_min = latMin
        self.lat_max = latMax
        self.lat_step = 6
        self.long_step = 6
        self.obstacle_length_limit = 10

        self.generateCoordinates()
        self.startTraversal()
        # self.handshake()

    ##------------------------------COORDINATE CREATION---------------------------##
    # Initializes all the coordinates based on the boundary and longitude/latitude steps
    # Adds coordinate (x,y) pairs [the keys], to the dictionary coordinates
    # Given and lat and longi, a coordinate is created if it in not in the dictionary. Then
    # the neighbors of (lat,longi) are created and put into the dictionary if not already.
    # TODO: @julie : this was the previous documentation ^^ check if we can delete

    # New idea: In this function, we initalize all the Coordinates in the graph,
    # and create two data structures:
    # 1) a list of Coordinates ordered in the proper lawnmower traversal
    # 2) a hashmap where the keys are the (x,y) tuple and values are the Coordinate instances
    # We don't need to keep track of the neigbor nodes or generate them initially because
    # later on we can generate them only for nodes where we detect an obstacle
    # and use the hashmap to look up the neighbors in constant time
    def generateCoordinates(self):
        """
        Initializes all the Coordinate objects in the graph based on boundary
        and longitude/latitute steps and creates 2 data structures:

        1) a list of Coordinates ordered based on lawnmower traversal search
        2) a dictionary where keys are the (x,y) tuple and values are
         Coordinate objects

        TODO: fix lawnmower to be zig zag and not c1 - cn ...
        """
        for lat in numpy.arange(
            self.lat_min, self.lat_max + self.lat_step, self.lat_step
        ):
            for long in numpy.arange(
                self.long_min, self.long_max + self.long_step, self.long_step
            ):
                # initialize list of entire traversal to be popped off queue
                self.traversalPath.append(Coordinate(lat, long))
                # initialize dict to be used to search for neighbors
                self.traversalDict[(lat, long)] = Coordinate(lat, long)

    ##-------------------------------ARDUINO CONNECTION-------------------------------##
    # def handshake(self):
    #     ser.write("start_pyserial")
    #     time.sleep(1)
    #     retrievedInput = readArduino()
    #     if (retrievedInput == "handshake_received"):
    #      return
    #      #print("Successful")
    #     else:
    #      raise Exception ("Handshake unsuccessful")

    # Parameter direction: string that tells us general direction to turn
    def sendHeading(self, direction):
        # ------------check this later--------------#
        self.ser.write(direction)

    def sendDataToArduino(self, data):
        self.ser.write(data)

    def readArduino(self):
        b = self.ser.readline()
        str_b = b.decode()
        str = str_b.strip()
        print(b)

    # change name to calculateDistance
    # cant change heading in place -> ask
    def moveRobotToCoordinate(self, coordinate, current_coordinate):
        # checkForObstacle(coordinate)

        # changes probability of coordinate to 1 since it's getting traversed
        coordinate.setTraversed()
        self.neighborRelativeToCoordinate(coordinate, current_coordinate)
        (next_x, next_y) = coordinate.getCoords()
        (current_x, current_y) = current_coordinate.getCoords()
        vector = Vector(current_x, current_y, next_x, next_y)
        distance = vector.getMagnitude()
        # how to actually calculate time?
        time = 1
        # how does robot know when to stop?
        self.ser.write(b"F")

        # tweek bc we won't(?) have diagonals

    def neighborRelativeToCoordinate(self, coordinate, current_coordinate):
        (next_x, next_y) = coordinate.getCoords()
        (current_x, current_y) = current_coordinate.getCoords()
        # all relative to robot

        if next_x > current_x and next_y > current_y:
            # rotate clockwise 45 degrees; topright
            self.sendHeading("top_right")
        elif next_x == current_x and next_y > current_y:
            # same heading; above
            self.sendHeading(b"F")
        elif next_x < current_x and next_y > current_y:
            # rotate counterclockwise 45 degrees; topleft
            self.sendHeading("top_left")
        elif next_x > current_x and next_y == current_y:
            # rotate clockwise 90 degrees; right
            self.sendHeading(b"R")
        elif next_x == current_x and next_y == current_y:
            # same heading
            return
            # raise Exception ("Current node = next node")
        elif next_x < current_x and next_y == current_y:
            # rotate counterclockwise 90 degrees; left
            self.sendHeading(b"L")
        elif next_x > current_x and next_y < current_y:
            # rotate clockwise 135 degrees; bottomright
            self.sendHeading("bottom_right")
        elif next_x == current_x and next_y < current_y:
            # rotate 180 degrees; below
            self.sendHeading(b"B")
        elif next_x < current_x and next_y < current_y:
            # rotate counterclockwise 135 degrees; bottomleft
            self.sendHeading("bottom_left")

    ##-------------------------------OBSTACLE AVOIDANCE/PATH-------------------------------##
    def check_for_obstacle(self, vertex):
        """Preliminary implementation:
        Checks if obstacle is present, and if it is,
        sets [vertex] as an obstacle Coordinate."""
        updated_distance_data = 30
        # self.retrieveDistance()
        for obstacle_attempt in range(0, 2):
            if (
                obstacle_attempt == 0
                and updated_distance_data < self.obstacle_length_limit
            ):
                time.sleep(10)
            elif (
                obstacle_attempt == 1
                and updated_distance_data < self.obstacle_length_limit
            ):
                vertex.setObstacle()

    # Precondition: Robot is at (lat_min, long_min)
    def obstacle_path(self):
        queue = self.traversalPath[:]  # make a copy of traversal list
        closed = []
        current_vertex = Coordinate(
            self.robot_starting_position[0], self.robot_starting_position[1]
        )

        while queue:
            vertex = queue.pop(0)  # Next node to visit

            if vertex.getProbability() == 1:  # if closed, then skip it
                continue

            self.check_for_obstacle(vertex)
            if vertex.getProbability() == 2:
                branched = encircle_obstacle(current_vertex, queue)
                obstacle_coordinates += branched
            (x, y) = vertex.getCoords()

            closed.append(vertex)
            self.moveRobotToCoordinate(vertex, current_vertex)
            # stop function

            current_vertex = vertex  # succssfully moed to desired node
        return rawCoordsTraversed

    # corner case: going back and forth repeatedly
    def encircle_obstacle(self, current_node, queue):
        # We want to get to the node after the blocked node
        branched_path = []  # nodes we travel to in this iteration
        goal_node = queue.pop(0)

        while current_node.getCoords() != goal_node.getCoords():
            choose_optimal_neighbor(current_node, goal_node)
            moveRobotToCoordinates(choose_optimal_neighbor.getCoords())
            # stop function
            branched_path.append(choose_optimal_neighbor)
            current_node = choose_optimal_neighbor
        return branched_path

    def get_distance(self, directional_node, goal_node):
        # dir_coords = directional_node.getCoords()
        # goal_coords = goal_node.getCoords()
        # return math.dist(dir_coords,goal_coords)
        x = directional_node.getX() - goal_node.getX()
        y = direction_node.getY() - goal_node.getY()
        inside = x**2 + y**2
        return math.sqrt(inside)

    def get_shortest_path(self, dir_lst, goal_node):
        distance_lst = []
        min_dist = Integer.MAX_VALUE
        for direction in dir_lst:
            dist = get_distance(direction, goal_node)
            if dist < min_dist:
                min_dist = dist
                distance_lst = [direction]
            elif dist == min_dist:
                distance_lst.append(direction)
        return distance_lst

    def get_lowest_prob(self, dir_lst):
        min_prob = dir_lst[0]
        min_prob_list = [dir_lst[0]]
        for direction in dir_lst:
            if direction.getProbability() < min_prob:
                min_prob = direction
                min_prob_list = [dir_lst]
            elif direction.getProbability() == min_prob:
                min_prob_list.append(direction)
        # If there is more than one element, they have the same
        # distance and priority, so we return the first element at random
        # May need to change in future implementations
        return min_prob_list[0]

    def choose_optimal_neighbor(self, current_node, goal_node):
        coords = current_node.getCoords()
        # future: add bound constrictions // think about realtivity

        left = traversalDict(coords[0] - self.lat_step, coords[1])
        right = traversalDict(coords[0] + self.lat_step, coords[1])
        front = traversalDict(coords[0], coords[1] - self.long_step)
        back = traversalDict(coords[0], coords[1] + self.long_step)

        directions = [front, left, back, right]
        for direction in directions:
            self.check_for_obstacle(direction)
            if direction.getProbability() == 2:
                directions.remove(direction)
            # turn robot left 90 degrees
            # self.change_heading(90)
            sendHeading(b"L")

        shortest_dir = self.get_shortest_path(directions)

        if len(shortest_dir) == 1:
            return shortest_dir[0]

        chosen_node = self.get_lowest_prob(shortest_dir)

        return chosen_node

        # Starts the traversal
        # Precondition: (x,y) must be a key value in dictionary coordinates

    def startTraversal(self):
        return self.obstacle_path()
