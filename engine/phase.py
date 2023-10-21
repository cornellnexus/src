from enum import Enum


class Phase(Enum):
    """
    An enumeration of different robot phases.
    """

    SETUP = 1
    TRAVERSE = 2
    AVOID_OBSTACLE = 3
    RETURN = 4
    DOCKING = 5
    COMPLETE = 6
    FAULT = 7
