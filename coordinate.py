
class Coordinate:
    x = 0
    y = 0
    neighbors = []
    # obstacle = False
    # traversed = False

    #probability:
    # 0: not traversed
    # 1: traversed
    # 2: obstacle
    probability = 0

    def __init__(self, x, y, neighbors):
        self.x = x
        self.y = y
        self.neighbors = neighbors

    def addNeighbor(self, neighbor):
        self.neighbors.append(neighbor)

    def setObstacle(self):
        self.probability = 2

    def setTraversed(self):
        self.probability = 1

    def getProbability(self):
        self.probability

    # def getTraversed(self):
    #     return self.traversed

    def getNeighbors(self):
        return self.neighbors
    #
    # def getObstacle(self):
    #     return self.obstacle

    def getCoords(self):
        return (self.x, self.y)

    def __eq__(self, other):
        if isinstance(other, Coordinate):
            return  self.x == other.x and self.y == other.y
        return False


    def __repr__(self):
        return "".join("("+str(self.x)+","+str(self.y)+")")

    def __str__(self):
        return "[(" + str(self.x) + ", " + str(self.y) + ") --> " + str(self.neighbors) + "]"
