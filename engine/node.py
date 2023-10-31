class Node:
    """Instances represent the position node of the robot's graph traversal.

    INSTANCE ATTRIBUTES:
    lat: latitude GPS coordinate (y) [float]
    long: longitude GPS coordinate (x) [float]

    x: x position (meters) [float]
    y: y position (meters) [float]

    status: current status of the node [int]
        [0 = not traversed, 1 = traversed, 2 = obstacle]

    is_active: whether the node is currently in use

    is_border: type of node on the grid
        [0 = not on the border of the grid, 1 = on the border of the grid]

    """

    def __init__(self, lat, long, x, y, is_border=False, active=False, status=0):
        self.lat = lat
        self.long = long
        self.x = x
        self.y = y
        self.is_border = is_border
        self.status = status
        self.is_active = active

    def get_m_coords(self):
        return (self.x, self.y)

    def get_gps_coords(self):
        return (self.lat, self.long)

    def is_border_node(self):
        return self.is_border

    def is_active_node(self):
        return self.is_active

    def activate_node(self):
        self.is_active = True

    def set_border_node(self):
        self.is_border = 1

    def get_status(self):
        return self.status

    def set_status(self, new_status):
        self.status = new_status

    def __eq__(self, other):
        if isinstance(other, Node):
            return (
                self.long == other.long
                and self.lat == other.lat
                and self.status == other.status
            )
        else:
            return False

    def __repr__(self):
        return "".join("(" + str(self.lat) + "," + str(self.long) + ")")
