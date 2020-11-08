
class Node:
    """ Instances represent a node of the robot's graph traversal.
        INSTANCE ATTRIBUTES:
        long: longitude coordinate [int]
        lat: latitude coordinate [int]
        status: current status of the node
            [0 = not traversed, 1 = traversed, 2 = obstacle] """

    def __init__(self, x, y, status = 0):
        self.long = x
        self.lat = y
        self.status = status

    def get_coords(self):
        return (self.long, self.lat)

    def get_status(self):
        return self.status
        
    def set_status(self, new_status):
        self.status = new_status

    def __eq__(self, other):
        if isinstance(other, Node):
            return  self.long == other.long and self.lat == other.lat and self.status == other.status
        else:
          return False

    def __repr__(self):
        return "".join("("+str(self.long)+","+str(self.lat)+")")

    # def __str__(self):
    #     return "[(" + str(self.long) + ", " + str(self.lat) + ") --> " + str(self.neighbors) + "]"

