import math
from engine.kinematics import get_vincenty_x, get_vincenty_y

class BaseStation:
    """
    Contains base-station-specific information.
    """

    def __init__(self, coord, grid, heading=math.pi/2, battery=None, plastic_load=None):
        """
        Arguments:
            coord: location of the base station in GPS coordinates in the form (latitude, longitude)
            grid: 
            heading: which direction the base station is facing in terms of unit circle (in radians), by
                default faces North (angle pi/2)
            battery: TODO
            plastic_load: TODO
        Instance Attributes:
            coord: location of the base station in GPS coordinates in the form (latitude, longitude)
            position: location of base station in the Grid coordinate system
            heading: which direction the base station is facing in terms of unit circle (in radians), by
                default faces North (angle pi/2)
            battery: TODO
            plastic_load: TODO
        """
        self.coord = coord
        x = get_vincenty_x((grid.lat_min, grid.long_min),
                            coord)
        y = get_vincenty_y((grid.lat_min, grid.long_min),
                            coord)
        self.position = (x, y)
        self.heading = heading
        self.battery = battery # Replace w/ battery reading
        self.plastic_load = plastic_load # Replace w/ base station bucket reading (depends on electrical)




