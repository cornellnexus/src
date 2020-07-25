
class Coordinate:
    x = 0
    y = 0
    neighbors = []
    obstacle = False
    traversed = False

    def __init__(self, x, y, neighbors):
        self.x = x
        self.y = y
        self.neighbors = neighbors

    def addNeighbor(self,neighbor):
        self.neighbors.append(neighbor)

    def setObstacle(self,obstacle):
        self.obstacle = obstacle

    def setTraversed(self,traversed):
        self.traversed = traversed

    def getTraversed(self):
        return self.traversed

    def getNeighbors(self):
        return self.neighbors

    def getObstacle(self):
        return self.obstacle

    def __repr__(self):
        return "".join("("+str(self.x)+","+str(self.y)+")")

    def __str__(self):
        return "[(" + str(self.x) + ", " + str(self.y) + ") --> " + str(self.neighbors) + "]"
