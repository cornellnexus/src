from Coordinate import Coordinate

coordinates = {}

class Graph:
    xMax = 3
    yMax = 3

    def performDFSAlgorithm(self,start):
        start.setTraversed(True)
        print(start)

        for neighbor in start.getNeighbors():
            if not neighbor.getTraversed() and not neighbor.getObstacle():
                self.performDFSAlgorithm(neighbor)

    def checkIfCoordinateInCoordsDictElseGenerateNode(self,x,y):
        if (x,y) in coordinates:
            print(str(x) + ", " + str(y) + " in dictionary")
            coordFound = coordinates.get((x,y))
            print("It's neighbors.." + str(coordFound.getNeighbors()))
            return coordinates.get((x,y))
        else:
            print(str(x) + ", " + str(y) + " not in dictionary")

            newCoordinateNode = Coordinate(x,y,[])
            coordinates[(x,y)] = newCoordinateNode
            return newCoordinateNode

    def addNewNeighborNodeToCoordinate(self,coordinateNode,neighborX,neighborY):
        neighborNode = self.checkIfCoordinateInCoordsDictElseGenerateNode(neighborX,neighborY)
        coordinateNode.addNeighbor(neighborNode)

    def createNeighborNodesForCoordinate(self,x,y):
        #we assume that x , y is already created.
        print("----------------------------------------------------------------")
        print("ADDING NEIGHBORS TO NODE : " + str(x) + ", " +str(y))
        coordinateToAddNeighborsTo = coordinates.get((x,y))

        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x,y+1) #Neighbor above
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x,y-1) #Neighbor below
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x+1,y) #Neighbor to the right
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x-1,y) #Neighbor to the left
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x+1,y+1) #Diagnol upper right neighbor
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x-1,y+1) #Diagnoal upper left neighbor
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x+1,y-1) #Diagnoal lower right
        self.addNewNeighborNodeToCoordinate(coordinateToAddNeighborsTo,x-1,y-1) #Diagnol lower left

        print("We have finished adding neighbors...all neighbors added are...")
        print(str(coordinateToAddNeighborsTo.getNeighbors()))

    def generateGraph(self):
        for x in range(0,self.xMax): #x
            for y in range(0,self.yMax): #y
                #print(x,y)
                if (x,y) not in coordinates:
                    coordinates[(x,y)] = Coordinate(x,y,[])
                self.createNeighborNodesForCoordinate(x,y)

    def printDict(self):
         for key in coordinates.keys():
             print(key, '->', coordinates[key])

g = Graph()
c1 = Coordinate(1,1,[])
c2 = Coordinate(1,2,[])
c3 = Coordinate(1,3,[])
c4 = Coordinate(1,4,[])

c5 = Coordinate(2,1,[])
c6 = Coordinate(2,2,[])
c7 = Coordinate(2,3,[])
c8 = Coordinate(2,4,[])

c1.addNeighbor(c2)
c1.addNeighbor(c5)
c1.addNeighbor(c6)

c2.addNeighbor(c1)
c2.addNeighbor(c6)
c2.addNeighbor(c3)
c2.addNeighbor(c5)
c2.addNeighbor(c7)

c3.addNeighbor(c2)
c3.addNeighbor(c4)
c3.addNeighbor(c7)
c3.addNeighbor(c8)
c3.addNeighbor(c6)

c4.addNeighbor(c3)
c4.addNeighbor(c8)
c4.addNeighbor(c7)

c5.addNeighbor(c1)
c5.addNeighbor(c2)
c5.addNeighbor(c6)

c6.addNeighbor(c5)
c6.addNeighbor(c7)
c6.addNeighbor(c2)
c6.addNeighbor(c1)
c6.addNeighbor(c3)

c7.addNeighbor(c6)
c7.addNeighbor(c8)
c7.addNeighbor(c3)
c7.addNeighbor(c2)
c7.addNeighbor(c4)

c8.addNeighbor(c7)
c8.addNeighbor(c4)
c8.addNeighbor(c3)

#g.performDFSAlgorithm(c1)
g.generateGraph()
print("----")
g.printDict()
print("-------")

debugDict= {}
def debug():
    onetwo = coordinates[(0,1)].getNeighbors()[1]
    print("-----")
    print(onetwo)
    onetwoDirect = coordinates[(0,0)]
    print(onetwoDirect)
    print(onetwo is onetwoDirect)
g.performDFSAlgorithm(coordinates[(1,1)])
debug()
