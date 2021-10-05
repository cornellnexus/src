import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import animation as animation
from matplotlib import patches as patch
import unittest

from engine.grid import *
from engine.kinematics import limit_cmds, feedback_lin, integrate_odom, get_vincenty_x, get_vincenty_y
from engine.node import *
from engine.pid_controller import PID
from engine.robot import *
from engine.sim_trajectory import *
from engine.user_utils import *

"""
New tests for traversal path 
"""


grid=Grid(42.444250, 42.444599, -76.483682, -76.483276)
allowed_dist_error=0.5