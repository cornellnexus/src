import math


class BaseStation:
    """
    Contains base-station-specific information.
    """

    def __init__(self, position, heading=math.pi/2, battery=None, plastic_load=None):
        """
        Arguments:
            position: location of the base station in GPS coordinates in the form (latitude, longitude)
            heading: which direction the base station is facing in terms of unit circle (in radians), by
                default faces North (angle pi/2)
            battery: TODO
            plastic_load: TODO
        """
        self.position = position
        self.heading = heading
        self.battery = battery
        self.plastic_load = plastic_load



