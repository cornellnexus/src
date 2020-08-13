# from Coordinate import Coordinate
# import tkinter as tk
# import tkinter
# import time

# class Graph:
#     xMax = 10
#     yMax = 10
#     coordinates = {}

#     canvas = ""
#     robot = ""
#     root = ""

#     def __init__(self, x, y):
#         self.xMax = x
#         self.yMax = y
#         self.generateGraph()
#         self.initializeGrid()

#     def performIterativeDFSAlgorithmm(self,start):
#         stack = [start]

#         while stack:
#             vertex = stack.pop()

#             if vertex.getTraversed():
#                 continue

#             vertex.setTraversed(True)
#             (x,y) = vertex.getCoords()
#             self.moveRobot(x,y)

#             for neighbor in vertex.getNeighbors():
#                 stack.append(neighbor)

#     def performRecursiveDFSAlgorithm(self,start):
#         #Do not use with thousand of nodes. will run into stack overflow.
#         start.setTraversed(True)
#         (x,y) = start.getCoords()
#         self.moveRobot(x,y)

#         for neighbor in start.getNeighbors():
#             if not neighbor.getTraversed() and not neighbor.getObstacle():
#                 self.performDFSAlgorithm(neighbor)

#     def checkIfCoordinateInCoordsDictElseGenerateNode(self,x,y):
#         if (x,y) in self.coordinates:
#             return self.coordinates.get((x,y))
#         else:
#             newCoordinateNode = Coordinate(x,y,[])
#             self.coordinates[(x,y)] = newCoordinateNode
#             return newCoordinateNode

#     def addNewNeighborNodeToCoordinate(self,coordinateNode,neighborX,neighborY):
#         if (neighborX < self.xMax and neighborX >= 0
#             and neighborY < self.yMax and neighborY > 0):
#             neighborNode = self.checkIfCoordinateInCoordsDictElseGenerateNode(neighborX,neighborY)
#             coordinateNode.addNeighbor(neighborNode)

#     def createNeighborNodesForCoordinate(self,x,y):
#         #we assume that x , y is already created.
#         coordinateToAddNeighborsTo = self.coordinates.get((x,y))

#         self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x,y+1) #Neighbor above
#         self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x,y-1) #Neighbor below
#         self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x+1,y) #Neighbor to the right
#         self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x-1,y) #Neighbor to the left
#         self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x+1,y+1) #Diagnol upper right neighbor
#         self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x-1,y+1) #Diagnoal upper left neighbor
#         self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x+1,y-1) #Diagnoal lower right
#         self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x-1,y-1) #Diagnol lower left


#     def generateGraph(self):
#         for x in range(0,self.xMax): #x
#             for y in range(0,self.yMax): #y
#                 #print(x,y)
#                 if (x,y) not in self.coordinates:
#                     self.coordinates[(x,y)] = Coordinate(x,y,[])
#                 self.createNeighborNodesForCoordinate(x,y)

#     def printAllCoordinates(self):
#          for key in self.coordinates.keys():
#              print(key, '->', self.coordinates[key])

#     def startDFSTraversalAtCoordinate(self,x,y):
#         self.performIterativeDFSAlgorithmm(self.coordinates[(x,y)])
#         print("main loop")
#         self.canvas.pack(fill =tk.BOTH, expand = True)
#         self.root.mainloop()


# #--------------- GUI Functions -----------------------------
#     def moveRobot(self,x,y):
#         self.canvas.coords(self.robot,x-5,y-5,x+5,y+5)
#         self.canvas.update()
#         self.canvas.after(1)

#     def initializeGrid(self):
#         self.root = tk.Tk()
#         self.canvas = tk.Canvas(self.root, height=500, width=500, bg='white')
#         self.canvas.pack(fill=tk.BOTH, expand=True)
#         self.canvas.bind('<Configure>', self.createGrid)
#         self.robot = self.canvas.create_rectangle(0, 0, 10, 10, fill="red")

#     def createGrid(event=None):
#         self.root = tk.Tk()
#         self.canvas = tk.Canvas(self.root, height=self.yMax, width=self.xMax, bg='white')
#         self.canvas.pack(fill=tk.BOTH, expand=True)
#         w = self.canvas.winfo_width() # Get current width of canvas
#         h = self.canvas.winfo_height() # Get current height of canvas
#         self.canvas.delete('grid_line') # Will only remove the grid_line

#         # Creates all vertical lines at intevals of 100
#         for i in range(0, w, 25):
#             self.canvas.create_line([(i, 0), (i, h)], tag='grid_line')

#         # Creates all horizontal lines at intevals of 100
#         for i in range(0, h, 25):
#             self.canvas.create_line([(0, i), (w, i)], tag='grid_line')


# g = Graph(500,500)
# g.startDFSTraversalAtCoordinate(1,1)

from Coordinate import Coordinate
import tkinter as tk
import tkinter
import time
import math
import numpy
import csv 

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

    def __init__(self, x, y):
        self.xMax = x
        self.yMax = y
        self.definingInputs()
        self.generateLongStep()
        self.generateLatStep()
        self.creatingCoordinates()
        #self.generateGraph()
        self.initializeGrid()

    def performIterativeDFSAlgorithmm(self,start):
        stack = [start]

        while stack:
            vertex = stack.pop()

            if vertex.getTraversed():
                continue

            vertex.setTraversed(True)
            (x,y) = vertex.getCoords()

            print("HERE")
            with open("DFS.txt", "a", newline='') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow([x,y])

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

    #Takes user inputs for bounds on latitude/longitude
    def definingInputs(self):
        try: 
            self.long_min = float(input("Enter minimum longitude: "))
            print(type(self.long_min))
            self.long_max = float(input("Enter maximum longitude: "))
            if self.long_max <= self.long_min:
                raise Exception ("The maximum longitude must be larger than the minimum longitude")
            self.lat_min = input ("Enter minimum latitude: ")
            self.lat_max = input("Enter maximum latitude: ")
            if self.lat_max <= self.lat_min:
                raise Exception ("The maximum latitude must be larger than the minimum latitude")
        except ValueError:
            print("Please enter a number")

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

    def creatingCoordinates(self):
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
        self.canvas.after(1)

    def initializeGrid(self):
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root, height=500, width=500, bg='white')
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind('<Configure>', self.createGrid)
        self.robot = self.canvas.create_rectangle(0, 0, 10, 10, fill="red")

    def createGrid(self,event=None):
        w = self.canvas.winfo_width() # Get current width of canvas
        h = self.canvas.winfo_height() # Get current height of canvas
        self.canvas.delete('grid_line') # Will only remove the grid_line

        # Creates all vertical lines at intevals of 100
        for i in range(0, w, 25):
            self.canvas.create_line([(i, 0), (i, h)], tag='grid_line')

        # Creates all horizontal lines at intevals of 100
        for i in range(0, h, 25):
            self.canvas.create_line([(0, i), (w, i)], tag='grid_line')


g = Graph(500,500)
g.startDFSTraversalAtCoordinate(0,0)

