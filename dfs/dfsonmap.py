from Coordinate import Coordinate
import tkinter as tk
import tkinter
import time
import numpy as np
#import pandas as pd
#import matplotlib.pyplot as plt
#from shapely.geometry import Polygon, Point
#import geopandas
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

    canvas = ""
    robot = ""
    root = ""

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

        self.initializeGrid() #Eventually move this to another class file.



    #We should insted print out a list of nodes in the order of traversal. This function is doing
    #too much.
    def performIterativeDFSAlgorithmm(self,start):
        stack = [start]

        while stack:
            vertex = stack.pop()

            if vertex.getTraversed():
                continue

            vertex.setTraversed(True)
            (x,y) = vertex.getCoords()
            print("DFS Traveled to: (" + str(x) + ", " + str(y)+")")


            with open("DFS.txt", "w+", newline='') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow([x,y])

            if not vertex.getObstacle():
                self.moveRobot(x,y)

            for neighbor in vertex.getNeighbors():
                stack.append(neighbor)

    def performRecursiveDFSAlgorithm(self,start):
        #Do not use with thousand of nodes. will run into stack overflow.
        start.setTraversed(True)
        (x,y) = start.getCoords()
        self.moveRobot(x,y)

        for neighbor in start.getNeighbors():
            if not neighbor.getTraversed() and not neighbor.getObstacle():
                self.performDFSAlgorithm(neighbor)

    def checkIfCoordinateInCoordsDictElseGenerateNode(self,x,y):
        if (x,y) in self.coordinates:
            return self.coordinates.get((x,y))
        else:
            newCoordinateNode = Coordinate(x,y,[])
            self.coordinates[(x,y)] = newCoordinateNode
            return newCoordinateNode

    def addNewNeighborNodeToCoordinate(self,coordinateNode,neighborX,neighborY):
        if (neighborX < self.xMax and neighborX >= 0
            and neighborY < self.yMax and neighborY > 0):
            neighborNode = self.checkIfCoordinateInCoordsDictElseGenerateNode(neighborX,neighborY)
            coordinateNode.addNeighbor(neighborNode)

    def createNeighborNodesForCoordinate(self,x,y):
        #we assume that x , y is already created.
        coordinateToAddNeighborsTo = self.coordinates.get((x,y))

        #
        print("LAT_STEP: " + str(self.lat_step))
        print("LONG_STEP: " + str(self.long_step))
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

    #Plots points onto graph from reading points on CSV file
    # def generateGraphUsingCSV(self):
    #     with open('longandlats.csv') as csv_file:
    #         csv_reader = csv.reader(csv_file, delimiter = ',')
    #         line_count = 0
    #         for row in csv_reader:
    #             if line_count == 0:
    #                 line_count+=1
    #             else:
    #                 x = float(row[0])
    #                 y = float(row[1])
    #                 if (x,y) not in self.coordinates:
    #                     self.coordinates[(x,y)] = Coordinate(x,y,[])
    #                 self.createNeighborNodesForCoordinate(x,y)


    def generateLongStep(self):
        self.long_min = float(self.long_min)
        self.long_max = float(self.long_max)

        long_diff = self.long_max - self.long_min
        long_10th = math.log10(long_diff)
        long_10th = math.floor(long_10th)
        long_base_unit = 10**long_10th
        self.long_step = long_base_unit / 2



    def generateLatStep(self):
        self.lat_min = float(self.lat_min)
        self.lat_max = float(self.lat_max)

        lat_diff = self.lat_max - self.lat_min
        lat_10th = math.log10(lat_diff)
        lat_10th = math.floor(lat_10th)
        lat_base_unit = 10**lat_10th
        self.lat_step = lat_base_unit / 2

    def generateCoordinates(self):
        #coord = []

        for lat in numpy.arange(self.lat_min, self.lat_max, self.lat_step):
            for longi in numpy.arange(self.long_min, self.long_max, self.long_step):
                # coord.append((lat,longi))
                print("Latitude: " + str(lat) + " Longitude: " + str(longi))
                if (lat, longi) not in self.coordinates:
                    self.coordinates[(lat,longi)] = Coordinate(lat,longi,[])
                self.createNeighborNodesForCoordinate(lat,longi)



    def printAllCoordinates(self):
         for key in self.coordinates.keys():
             print(key, '->', self.coordinates[key])

    def startDFSTraversalAtCoordinate(self,x,y):
        self.performIterativeDFSAlgorithmm(self.coordinates[(x,y)])
        print("main loop")
        self.canvas.pack(fill =tk.BOTH, expand = True)
        self.root.mainloop()


#--------------- GUI Functions -----------------------------
    def moveRobot(self,x,y):
        self.canvas.coords(self.robot,x-5,y-5,x+5,y+5)
        self.canvas.update()
        self.canvas.after(300)

    def initializeGrid(self):
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root, height=self.yMax, width=self.xMax, bg='white')
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind('<Configure>', self.createGrid)
        self.robot = self.canvas.create_rectangle(0, 0, 5, 5, fill="red")

    def initializeMap(self):
        df = pd.read_csv("longandlats.csv")

        # BBox = ((df.longitude.min(),   df.longitude.max(),
        #          df.latitude.min(), df.latitude.max()))
        # #         > (42.4596, 42.4642, -76.5119, -76.5013)

        BBox = [-76.5119, -76.5013, 42.4596, 42.4642]

        ruh_m = plt.imread('map.png')

        fig, ax = plt.subplots(figsize=(8, 7))
        ax.scatter(df.longitude, df.latitude, zorder=1, alpha=1, c='r', s=10)
        ax.set_title('Cayuga Lake Shore')
        ax.set_xlim(BBox[0], BBox[1])
        ax.set_ylim(BBox[2], BBox[3])


        ax.imshow(ruh_m, zorder=0, extent=BBox, aspect='equal')

        # polys1 = geopandas.GeoSeries([Polygon([(-76.5033,42.4636),(-76.5023,42.4641),(-76.5013,42.4646),(-76.5113,42.4601)])])
        # df1 = geopandas.GeoDataFrame({'geometry': polys1, 'df1':[-76.5028,42.4638]})
        # ax1 = df1.plot(color='blue')

        plt.show()


    def createGrid(self,event=None):
        w = self.canvas.winfo_width() # Get current width of canvas
        h = self.canvas.winfo_height() # Get current height of canvas
        self.canvas.delete('grid_line') # Will only remove the grid_line

        # Creates all vertical lines at intevals of 100
        for i in range(0, w, 10):
            self.canvas.create_line([(i, 0), (i, h)], tag='grid_line')

        # Creates all horizontal lines at intevals of 100
        for i in range(0, h, 10):
            self.canvas.create_line([(0, i), (w, i)], tag='grid_line')


