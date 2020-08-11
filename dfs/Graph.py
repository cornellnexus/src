from Coordinate import Coordinate
import tkinter as tk
import tkinter
import time

class Graph:
    xMax = 10
    yMax = 10
    coordinates = {}

    canvas = ""
    robot = ""
    root = ""

    def __init__(self, x, y):
        self.xMax = x
        self.yMax = y
        self.generateGraph()
        self.initializeGrid()

    def performIterativeDFSAlgorithmm(self,start):
        stack = [start]

        while stack:
            vertex = stack.pop()

            if vertex.getTraversed():
                continue

            vertex.setTraversed(True)
            (x,y) = vertex.getCoords()
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

        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x,y+1) #Neighbor above
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x,y-1) #Neighbor below
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x+1,y) #Neighbor to the right
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x-1,y) #Neighbor to the left
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x+1,y+1) #Diagnol upper right neighbor
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x-1,y+1) #Diagnoal upper left neighbor
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x+1,y-1) #Diagnoal lower right
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x-1,y-1) #Diagnol lower left


    def generateGraph(self):
        for x in range(0,self.xMax): #x
            for y in range(0,self.yMax): #y
                #print(x,y)
                if (x,y) not in self.coordinates:
                    self.coordinates[(x,y)] = Coordinate(x,y,[])
                self.createNeighborNodesForCoordinate(x,y)

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

    def createGrid(event=None):
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root, height=self.yMax, width=self.xMax, bg='white')
        self.canvas.pack(fill=tk.BOTH, expand=True)
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
g.startDFSTraversalAtCoordinate(1,1)
