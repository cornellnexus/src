from Coordinate import Coordinate
import tkinter as tk
import tkinter
import time
import numpy as np
import csv
import math
import numpy


class Graph:
    xMax = 10
    yMax = 10
    coordinates = {}

    lat_min = 0
    lat_max = 0
    long_min = 0
    long_max = 0

    long_step = 0
    lat_step = 0

    def __init__(self, longMin, longMax, latMin, latMax):
        self.xMax = longMax #Graph boundary on x-axis
        self.yMax = latMax  #Graph boundary on y-axis
        
        self.long_min = longMin
        self.long_max = longMax
        self.lat_min = latMin
        self.lat_max = latMax

        self.generateLongStep()
        self.generateLatStep()
        self.generateCoordinates()


    #We should insted print out a list of nodes in the order of traversal. This function is doing
    #too much.
    def performIterativeDFSAlgorithmm(self,start):
        stack = [start]
        rawCoordsTraversed = []

        while stack:
            vertex = stack.pop()

            if vertex.getTraversed():
                continue

            vertex.setTraversed(True)
            (x,y) = vertex.getCoords()
            print("X AND Y" + str((x,y)))

            if not vertex.getObstacle():
                #print("NOT OBSTACLE")
                rawCoordsTraversed.append((x,y))

            for neighbor in vertex.getNeighbors():
                stack.append(neighbor)
                print("Neighbor: " + str(neighbor))
            
        return rawCoordsTraversed

    def checkIfCoordinateInCoordsDictElseGenerateNode(self,x,y):
        #print("here 3")
        if (x,y) in self.coordinates:
            return self.coordinates.get((x,y))
        else:
            #print("GENERATING NODE...")
            newCoordinateNode = Coordinate(x,y,[])
            self.coordinates[(x,y)] = newCoordinateNode
            return newCoordinateNode

    def addNewNeighborNodeToCoordinate(self,coordinateNode,neighborX,neighborY):
        #print("Here 2")
        if (neighborX < self.lat_max and neighborX > self.lat_min
            and neighborY < self.long_max and neighborY > self.long_min):
            #print("here 4")
            neighborX = round(neighborX,5)
            neighborY = round(neighborY,5)
            neighborNode = self.checkIfCoordinateInCoordsDictElseGenerateNode(neighborX,neighborY)
            coordinateNode.addNeighbor(neighborNode)
            print("COORDINATE NODES NEIGHBORS: ")
            #print("In Bounds")
        #else: 
            #print("Oout of Bouds: " + str(neighborX) + ", " +  str(neighborY))

    def createNeighborNodesForCoordinate(self,x,y):
        #we assume that x , y is already created.
        coordinateToAddNeighborsTo = self.coordinates.get((x,y))

        #print("------ " + str(x) + ", "+ str(y))
        print("lat step: " + str(self.lat_step))
        print("long step: " + str(self.long_step))

        #self.lat_step = round(self.lat_step,5)
        #self.long_step = round(self.long_step,5)

        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x,y+self.lat_step) #Neighbor above
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x,y-self.lat_step) #Neighbor below
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x+self.long_step,y) #Neighbor to the right
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x-self.long_step,y) #Neighbor to the left
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x+self.long_step,y+self.lat_step) #Diagnol upper right neighbor
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x-self.long_step,y+self.lat_step) #Diagnoal upper left neighbor
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x+self.long_step,y-self.lat_step) #Diagnoal lower right
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x-self.long_step,y-self.lat_step) #Diagnol lower left


    def generateGraph(self):
        for x in range(0,self.xMax): #x
            for y in range(0,self.yMax): #y
                #print(x,y)
                if (x,y) not in self.coordinates:
                    self.coordinates[(x,y)] = Coordinate(x,y,[])
                self.createNeighborNodesForCoordinate(x,y)

    def generateLongStep(self):
        self.long_min = float(self.long_min)
        self.long_max = float(self.long_max)

        # long_diff = self.long_max - self.long_min
        # long_10th = math.log10(long_diff)
        # long_10th = math.floor(long_10th)
        # long_base_unit = 10**long_10th
        # self.long_step = long_base_unit / 2
        self.long_step = round((self.long_max - self.long_min) / 10,5)
        #print("Long_step = " + str(self.long_step))

    def generateLatStep(self):
        self.lat_min = float(self.lat_min)
        self.lat_max = float(self.lat_max)

        # lat_diff = self.lat_max - self.lat_min
        # lat_10th = math.log10(lat_diff)
        # lat_10th = math.floor(lat_10th)
        # lat_base_unit = 10**lat_10th
        # self.lat_step = lat_base_unit / 2

        self.lat_step = round((self.lat_max - self.lat_min) / 10,5)
        #print("Lat_step = " + str(self.lat_step))

    def generateCoordinates(self):
        #coord = []

        for lat in numpy.arange(self.lat_min, self.lat_max, self.lat_step):
            for longi in numpy.arange(self.long_min, self.long_max, self.long_step):
                # coord.append((lat,longi))
                
                lat = round(lat,5)
                longi = round(longi,5)

                print("Latitude: " + str(lat) + " Longitude: " + str(longi))
                if (lat, longi) not in self.coordinates:
                    self.coordinates[(lat,longi)] = Coordinate(lat,longi,[])
                self.createNeighborNodesForCoordinate(lat,longi)



    def printAllCoordinates(self):
         for key in self.coordinates.keys():
             print(key, '->', self.coordinates[key])

    def startDFSTraversalAtCoordinate(self,x,y):
        print("STARTING DFS TRAVERSAL")
        return self.performIterativeDFSAlgorithmm(self.coordinates[(x,y)])
        
