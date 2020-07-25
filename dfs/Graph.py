from Coordinate import Coordinate

coordinates = {}

class Graph:
    xMax = 10
    yMax = 10

    def __init__(self, x, y):
        self.xMax = x
        self.yMax = y

    def performDFSAlgorithm(self,start):
        start.setTraversed(True)
        print(start)

        for neighbor in start.getNeighbors():
            if not neighbor.getTraversed() and not neighbor.getObstacle():
                self.performDFSAlgorithm(neighbor)

    def checkIfCoordinateInCoordsDictElseGenerateNode(self,x,y):
        if (x,y) in coordinates:
            return coordinates.get((x,y))
        else:
            newCoordinateNode = Coordinate(x,y,[])
            coordinates[(x,y)] = newCoordinateNode
            return newCoordinateNode

    def addNewNeighborNodeToCoordinate(self,coordinateNode,neighborX,neighborY):
        if (neighborX < self.xMax and neighborX >= 0
            and neighborY < self.yMax and neighborY > 0):
            neighborNode = self.checkIfCoordinateInCoordsDictElseGenerateNode(neighborX,neighborY)
            coordinateNode.addNeighbor(neighborNode)

    def createNeighborNodesForCoordinate(self,x,y):
        #we assume that x , y is already created.
        coordinateToAddNeighborsTo = coordinates.get((x,y))

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
                if (x,y) not in coordinates:
                    coordinates[(x,y)] = Coordinate(x,y,[])
                self.createNeighborNodesForCoordinate(x,y)

    def printAllCoordinates(self):
         for key in coordinates.keys():
             print(key, '->', coordinates[key])

g = Graph(10,10)

g.generateGraph()

g.performDFSAlgorithm(coordinates[(1,1)])
