
class Coordinate:
    """ Instances represent a node of the robot's graph traversal.
        INSTANCE ATTRIBUTES:
        x: x coordinate [int]
        y: y coordinate [int]
        probability: type of the node
            [0 = not traversed, 1 = traversed, 2 = obstacle] """
    x = 0
    y = 0
    probability = 0

    def __init__(self, x, y, probability = 0):
        self.x = x
        self.y = y
        self.probability = probability

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getCoords(self):
        return (self.x, self.y)

    def getProbability(self):
        self.probability

    def setObstacle(self):
        self.probability = 2

    def setTraversed(self):
        self.probability = 1

    def __eq__(self, other):
        if isinstance(other, Coordinate):
            return  self.x == other.x and self.y == other.y
        return False

    def __repr__(self):
        return "".join("("+str(self.x)+","+str(self.y)+")")

    def __str__(self):
        return "[(" + str(self.x) + ", " + str(self.y) + ") --> " + str(self.neighbors) + "]"

    # ******************** OLD STUFF ******************************************
    # def addNeighbor(self, neighbor):
    #     self.neighbors.append(neighbor)

    # def getTraversed(self):
    #     return self.traversed

    # def getNeighbors(self):
    #     return self.neighbors
    #
    # def getObstacle(self):
    #     return self.obstacle
