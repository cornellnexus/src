
class Node:
    """ Instances represent a node of the robot's graph traversal.
        INSTANCE ATTRIBUTES:
        long: longitude coordinate [int]
        lat: latitude coordinate [int]
        status: current status of the node
            [0 = not traversed, 1 = traversed, 2 = obstacle] """

    def __init__(self, y, x, status = 0):
        self.lat = y
        self.long = x
        self.status = status

    def get_coords(self):
        return (self.lat, self.long)

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
        return "".join("("+str(self.lat)+","+str(self.long)+")")



    # def __str__(self):
    #     return "[(" + str(self.long) + ", " + str(self.lat) + ") --> " + str(self.neighbors) + "]"
