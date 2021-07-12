from CoordinateNina import CoordinateNina
import tkinter as tk
import tkinter
import time
import numpy as np
import csv
import math
import numpy


class Graph:

    #xMax = 10
    #yMax = 10

    # Dictionary to hold the created coordinates. Key is an (x,y) pair, and value
    # is a coordinate object.
    coordinates = {}

    # Declaration of latitude min/max, and longitude min/max. These will be assigned
    # by user input
    lat_min = 0
    lat_max = 0
    long_min = 0
    long_max = 0

    # Declaration of longitude step and latitude step. These are determined based on
    # the differences between longitude max/min and latitude max/min, respectuflly.
    long_step = 0
    lat_step = 0

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

        self.generateLongStep()
        self.generateLatStep()
        self.generateCoordinates()

    # We should insted print out a list of nodes in the order of traversal. This function is doing
    # too much.

    # Iterative DFS Algorithm
    # Precondition: Start must be a coordinate in the dictionary coordinates
    # Preforms DFS as expected, adding the nontraversed neighbors of the recent nodes to the stack.
    def performIterativeDFSAlgorithmm(self, start):
        stack = [start]
        rawCoordsTraversed = []

        while stack:
            vertex = stack.pop()

            if vertex.getTraversed():
                continue

            vertex.setTraversed(True)
            (x, y) = vertex.getCoords()
            #print("X AND Y" + str((x,y)))

            if not vertex.getObstacle():
                #print("NOT OBSTACLE")
                rawCoordsTraversed.append((x, y))

            for neighbor in vertex.getNeighbors():
                stack.append(neighbor)
                #print("Neighbor: " + str(neighbor))

        return rawCoordsTraversed

    def DFSStart(self, start):
        stack = [start]
        rawCoordsTraversed = []

        vertex = stack.pop()

        if not(vertex.getTraversed()):
            vertex.setTraversed(True)
            (x, y) = vertex.getCoords()

            if not vertex.getObstacle():
                rawCoordsTraversed.append((x, y))

            for neighbor in vertex.getNeighbors():
                stack.append(neighbor)

        return (rawCoordsTraversed, stack)

    #prevStep = (rawCoordsTraversed, stack)

    def DFSStep(self, prevStep):
        stack = prevStep[1]
        rawCoordsTraversed = prevStep[0]

        if len(stack) == 0:
            return (rawCoordsTraversed, ["DONE"])
            # no more nodes left to traverse
        else:
            vertex = stack.pop()

            if not(vertex.getTraversed()):
                vertex.setTraversed(True)
                (x, y) = vertex.getCoords()

                if not vertex.getObstacle():
                    rawCoordsTraversed.append((x, y))

                for neighbor in vertex.getNeighbors():
                    stack.append(neighbor)

            return (rawCoordsTraversed, stack)

    # Checks if a coordinate is already in the dictionary coordinates
    # If the coordinate is in the dictionary, it's value is returned
    # If the coordinate is not in the dictionary, then a coordinate object is
    # created using parameters x and y, and the value of the new object is returned
    # Precondition: x and y are floats
    def checkIfCoordinateInCoordsDictElseGenerateNode(self, x, y):
        #print("here 3")
        if (x, y) in self.coordinates:
            print("NEIGHBOR NODE FOUND")
            return self.coordinates.get((x, y))
        else:
            print("GENERATING NODE...")
            newCoordinateNode = CoordinateNina(x, y, [])
            self.coordinates[(x, y)] = newCoordinateNode
            return newCoordinateNode

    # Adds a neighbor to the parameter coordinateNode if neighborX and neighborY
    # are not out of the boundary.
    # If the neighbor node is not in the dictionary, a coordinate object is created,
    # added to the dictionary, and then added to the neighbor array of coordinateNode
    # If the neighbor node is already in the dictionary, the coordinate is added to
    # the neighbor array of coordinateNode.
    # Precondition: coordinateNode is in the dictionary coordinates
    def addNewNeighborNodeToCoordinate(self, coordinateNode, neighborX, neighborY):
        if (neighborX <= self.lat_max and neighborX >= self.lat_min
                and neighborY <= self.long_max and neighborY >= self.long_min):
            neighborX = round(neighborX, 5)
            neighborY = round(neighborY, 5)

            print("NEIGHBOR NODE - Latitude: " +
                  str(neighborX) + " Longitude: " + str(neighborY))

            neighborNode = self.checkIfCoordinateInCoordsDictElseGenerateNode(
                neighborX, neighborY)
            coordinateNode.addNeighbor(neighborNode)
        # else:
            #print("Oout of Bouds: " + str(neighborX) + ", " +  str(neighborY))

    # Creates neighbors for a single node
    # Precondition: the coordinate at (x,y) is already created and can be found in the
    # dictionary coordinates
    # This method accounts for all 8 possible neighbors of a node. If a node is on an
    # edge or a corner, addNewNeighborNodeToCoordinate will take this into account and
    # will not add a node that does not fit in the given boundary.
    def createNeighborNodesForCoordinate(self, x, y):
        # we assume that x , y is already created.
        coordinateToAddNeighborsTo = self.coordinates.get((x, y))

        print("lat step: " + str(self.lat_step))
        print("long step: " + str(self.long_step))

        #self.lat_step = round(self.lat_step,5)
        #self.long_step = round(self.long_step,5)

        self.addNewNeighborNodeToCoordinate(
            coordinateToAddNeighborsTo, x, y+self.long_step)  # Neighbor above
        self.addNewNeighborNodeToCoordinate(
            coordinateToAddNeighborsTo, x, y-self.long_step)  # Neighbor below
        self.addNewNeighborNodeToCoordinate(
            coordinateToAddNeighborsTo, x+self.lat_step, y)  # Neighbor to the right
        self.addNewNeighborNodeToCoordinate(
            coordinateToAddNeighborsTo, x-self.lat_step, y)  # Neighbor to the left
        self.addNewNeighborNodeToCoordinate(
            coordinateToAddNeighborsTo, x+self.lat_step, y+self.long_step)  # Diagnol upper right neighbor
        self.addNewNeighborNodeToCoordinate(
            coordinateToAddNeighborsTo, x-self.lat_step, y+self.long_step)  # Diagnoal upper left neighbor
        self.addNewNeighborNodeToCoordinate(
            coordinateToAddNeighborsTo, x+self.lat_step, y-self.long_step)  # Diagnoal lower right
        self.addNewNeighborNodeToCoordinate(
            coordinateToAddNeighborsTo, x-self.lat_step, y-self.long_step)  # Diagnol lower left

    # Auto-creates coordinates given xMax and yMax. This is our old hard-coded version
    # of created the coordinates.
    # def generateGraph(self):
    #     for x in range(0,self.xMax): #x
    #         for y in range(0,self.yMax): #y
    #             #print(x,y)
    #             if (x,y) not in self.coordinates:
    #                 self.coordinates[(x,y)] = Coordinate(x,y,[])
    #             self.createNeighborNodesForCoordinate(x,y)

    # Initializes longitude step based on the user inputs longitude min/max
    def generateLongStep(self):
        self.long_min = float(self.long_min)
        self.long_max = float(self.long_max)

        # long_diff = self.long_max - self.long_min
        # long_10th = math.log10(long_diff)
        # long_10th = math.floor(long_10th)
        # long_base_unit = 10**long_10th
        # self.long_step = long_base_unit / 2
        self.long_step = round((self.long_max - self.long_min) / 9, 5)
        #print("Long_step = " + str(self.long_step))

    # Initializes latitude step based on the user inputs latitude min/max
    def generateLatStep(self):
        self.lat_min = float(self.lat_min)
        self.lat_max = float(self.lat_max)

        # lat_diff = self.lat_max - self.lat_min
        # lat_10th = math.log10(lat_diff)
        # lat_10th = math.floor(lat_10th)
        # lat_base_unit = 10**lat_10th
        # self.lat_step = lat_base_unit / 2

        self.lat_step = round((self.lat_max - self.lat_min) / 9, 5)
        #print("Lat_step = " + str(self.lat_step))

    # Initializes all the coordinates based on the boundary and longitude/latitude steps
    # Adds coordinate (x,y) pairs [the keys], to the dictionary coordinates
    # Given and lat and longi, a coordinate is created if it in not in the dictionary. Then
    # the neighbors of (lat,longi) are created and put into the dictionary if not already.
    def generateCoordinates(self):
        #coord = []

        for lat in numpy.arange(self.lat_min, self.lat_max+self.lat_step, self.lat_step):
            for longi in numpy.arange(self.long_min, self.long_max+self.long_step, self.long_step):
                # coord.append((lat,longi))

                lat = round(lat, 5)
                longi = round(longi, 5)

                print("------------------------OUTER COORD - Latitude: " +
                      str(lat) + " Longitude: " + str(longi))
                if (lat, longi) not in self.coordinates:
                    print("OUTER COORD NOT FOUND")
                    self.coordinates[(lat, longi)] = CoordinateNina(
                        lat, longi, [])
                else:
                    print("OUTER COORD FOUND")

                self.createNeighborNodesForCoordinate(lat, longi)

        print(len(self.coordinates.keys()))

        print(self.coordinates)

    # Returns all of the coordinates (x,y) keys as a list
    def getAllCoordinatesGenerated(self):
        return list(self.coordinates.keys())

    # Prints all of the coordinates in the dictionary
    def printAllCoordinates(self):
        for key in self.coordinates.keys():
            print(key, '->', self.coordinates[key])

    # Starts the DFS traversal
    # Precondition: (x,y) must be a key value in dictionary coordinates
    def startDFSTraversalAtCoordinate(self, x, y):
        print("STARTING DFS TRAVERSAL: " + str(x) + ", " + str(y))
        return self.performIterativeDFSAlgorithmm(self.coordinates[(x, y)])


# Starts the DFS traversal
    # Precondition: (x,y) must be a key value in dictionary coordinates


    def startDFSNina(self, x, y):
        print("STARTING DFS TRAVERSAL: " + str(x) + ", " + str(y))
        return self.DFSStart(self.coordinates[(x, y)])
