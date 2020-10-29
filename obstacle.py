from coordinate import Coordinate
import time
import numpy as np
import csv
import math
import numpy
import serial
from Vector import Vector

class Obstacle:
    #Dictionary to hold the created coordinates. Key is an (x,y) pair, and value
    #is a coordinate object.
    traversalPath = []
    traversalcoordinates = {}
    # obstacle_coordinates = {}

    #Declaration of latitude min/max, and longitude min/max. These will be assigned
    #by user input
    lat_min = 0
    lat_max = 0
    long_min = 0
    long_max = 0
    #Declaration of longitude step and latitude step. These are determined based on
    #the differences between longitude max/min and latitude max/min, respectuflly.
    # 2 times the width of the robot (future implementations)
    long_step = 0
    lat_step = 0

    ser = serial.Serial('/dev/cu.usbserial- 1420', 9600)

    #Time to wait before checking for distance data again.
    distance_check_wait_time = 5

    #in the future, probably location of base station
    robot_starting_position = (lat_min,long_min)

    #Initialization of variables
    #User inputs longitude min/max and latitude min/max for Initialization
    #Based on user inputs, the long step and latitude step are created
    #With the user inputs and steps, the coordinates are created
    def __init__(self, longMin, longMax, latMin, latMax):
        #self.xMax = longMax #Graph boundary on x-axis
        #self.yMax = latMax  #Graph boundary on y-axis

        self.long_min = longMin
        self.long_max = longMax
        self.lat_min = latMin
        self.lat_max = latMax
        self.lat_step = 6
        self.long_step = 6

        self.generateCoordinates()
        self.handshake()

##-------------------------------COORDINATE CREATION-------------------------------##
    #Initializes all the coordinates based on the boundary and longitude/latitude steps
    #Adds coordinate (x,y) pairs [the keys], to the dictionary coordinates
    #Given and lat and longi, a coordinate is created if it in not in the dictionary. Then
    # the neighbors of (lat,longi) are created and put into the dictionary if not already.

    # New idea: In this function, we initalize all the Coordinates in the graph, and create two data structures:
    # 1) a list of Coordinates ordered in the proper lawnmower traversal
    # 2) a hashmap where the keys are the (x,y) tuple and values are the Coordinate instances
    # We don't need to keep track of the neigbor nodes or generate them initially because later on we can generate them only for nodes where we detect an obstacle and use the hashmap to look up the neighbors in constant time
    def generateCoordinates(self):
        for lat in numpy.arange(self.lat_min, self.lat_max+self.lat_step, self.lat_step):
            for long in numpy.arange(self.long_min, self.long_max+self.long_step, self.long_step):
                # initialize list of entire traversal to be popped off queue
                self.traversalPath.append(Coordinate(lat,long))
                # initialize dict to be used to search for neighbors
                self.traversalDict[(lat,long)] = Coordinate(lat,long)

##-------------------------------ARDUINO CONNECTION-------------------------------##
    def handshake(self):
        Serial.print("start_pyserial")
        time.sleep(1)
        retrievedInput = readArduino()
        if (retrievedInput == "handshake_received"):
         return
         #print("Successful")
        else:
         raise (Exception "Handshake unsuccessful")

    #Parameter direction: string that tells us general direction to turn
    def sendHeading(self, direction):
        #------------check this later--------------#
        Serial.print(direction)

    def sendDataToArduino(self, data):
        Serial.print(data)

    def readArduino (self):
        b = ser.readline()
        str_b = b.decode()
        str = str_b.strip()
        print(b)

    #change name to calculateDistance
    #cant change heading in place -> ask
    def moveRobotToCoordinate(self, coordinate, current_coordinate):
        # checkForObstacle(coordinate)
        (next_x,next_y) = coordinate.getCoords()
        (current_x,current_y) = current_coordinate.getCoords()
        vector = Vector(current_x, current_y, next_x, next_y)
        distance = vector.getMagnitude()
        #how to actually calculate time?
        time = 1
        Serial.print("move_forward " + str(time))

        #tweek bc we won't(?) have diagonals
    def neighborRelativeToCoordinate(self,coordinate, current_coordinate):
        (next_x,next_y) = coordinate.getCoords()
        (current_x,current_y) = current_coordinate.getCoords()
        #all relative to robot

        if (next_x > current_x and next_y > current_y):
            #rotate clockwise 45 degrees; topright
            sendHeading("top_right")
        elif (next_x == current_x and next_y > current_y):
            #same heading; above
            sendHeading("above")
        elif (next_x < current_x and next_y > current_y):
            #rotate counterclockwise 45 degrees; topleft
            sendHeading("top_left")
        elif (next_x > current_x and next_y == current_y):
            #rotate clockwise 90 degrees; right
            sendHeading("right")
        elif (next_x == current_x and next_y == current_y):
            #same heading
            raise Exception ("Current node = next node")
        elif (next_x < current_x and next_y == current_y):
            #rotate counterclockwise 90 degrees; left
            sendHeading("left")
        elif (next_x > current_x and next_y < current_y):
            #rotate clockwise 135 degrees; bottomright
            sendHeading("bottom_right")
        elif (next_x == current_x and next_y < current_y):
            #rotate 180 degrees; below
            sendHeading("below")
        elif (next_x < current_x and next_y < current_y):
            #rotate counterclockwise 135 degrees; bottomleft
            sendHeading("bottom_left")

##-------------------------------OBSTACLE AVOIDANCE/PATH-------------------------------##
    def check_for_obstacle(self,vertex):
        updated_distance_data = self.retrieveDistance()
        for obstacle_attempt in range(0,2):
            if (obstacle_attempt == 0 and updated_distance_data
               < obstacle_length_limit):
                time.sleep(10)
            elif (obstacle_attempt == 1 and updated_distance_data
                 < obstacle_length_limit):
                vertex.setObstacle()

    #Precondition: Robot is at (lat_min, long_min)
    def obstacle_path(self):
        queue = self.traversalPath[:] #make a copy
        closed = []
        current_vertex = robot_starting_position

        while queue:
            vertex = queue.pop() #Next node to visit

            if vertex.getProbability() == 1: # if closed, then skip it
                continue

            self.check_for_obstacle(vertex)
            if vertex.getProbability() == 2:
                encircle_obstacle(current_vertex, queue)
            (x,y) = vertex.getCoords()

            closed.append(vertex)
            self.moveRobotToCoordinate(vertex, current_vertex)

            current_vertex = vertex # succssfully moed to desired node
        return rawCoordsTraversed

    def encircle_obstacle(self,current_Node,queue):
        #We want to get to the node after the blocked node

        branchedPath = [] #nodes we travel to in this iteration
        goalNode = queue.pop()
        chooseOptimalNeighbor(current_Node, goalNode)

    def chooseOptimalNeighbor(current_Node, goalNode):
        left =
        right =
        front =
        back =
