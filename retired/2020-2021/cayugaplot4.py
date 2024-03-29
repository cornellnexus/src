"""
cayugaplot4 is the current working version of cayuga plot. As of now, it
plots the robot's location and heading using an arbitrarily defined animate 
function. It uses matplotlib wedges and circles and the funcAnimation 
feature to plot. 
The goal is to 
1) rewrite the animate function so that the robot's state
   is plotted according to the Kalman Filter output, and 
2) embed this graph into the GUI. 
"""

import numpy as np
import pandas as pd
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation as animation
from matplotlib import patches as patch
from matplotlib.widgets import Button
from user_utils import *

"""
When running this file, input 0, 10, 0, 10 for the corresponding values
below. get_coord_inputs() is documented in UserUtils.py
"""
longMin, longMax, latMin, latMax = get_coord_inputs()

"""
Set up window
"""
BoundaryBox = [longMin, longMax, latMin, latMax]
ruh_m = plt.imread("map.png")
fig, ax = plt.subplots(figsize=(8, 7))
ax.set_title("Cayuga Lake Shore")
ax.set_xlim(BoundaryBox[0], BoundaryBox[1])
ax.set_ylim(BoundaryBox[2], BoundaryBox[3])
ax.imshow(ruh_m, zorder=0, extent=BoundaryBox, aspect="equal")
fig.set_dpi(100)
fig.set_size_inches(7, 6.5)

"""
Create circle patch object to represent moving robot, and wedge patch for
robot's heading
"""
circle_patch = plt.Circle((5, 5), 0.1, fc="black")
wedge_patch = patch.Wedge(
    (5, 1), 1, 30, 50, animated=True, fill=False, width=0.9, ec="r", hatch="xx"
)


def init():
    circle_patch.center = (5, 5)
    ax.add_patch(circle_patch)
    # ax.add_patch(arc_patch)
    ax.add_patch(wedge_patch)
    return circle_patch, wedge_patch


"""
Update circle and wedge patch poses arbitrarily with each time step, to 
simulate movement
"""


def animate(i):
    x, y = circle_patch.center
    x = 5 + 3 * np.sin(np.radians(i))
    y = 5 + 3 * np.cos(np.radians(i))
    circle_patch.center = (x, y)
    wedge_patch.update({"center": [x, y]})
    wedge_patch.theta1 += 0.5 * x
    wedge_patch.theta2 += 0.5 * x
    wedge_patch.theta1 = wedge_patch.theta1 % 360
    wedge_patch.theta2 = wedge_patch.theta2 % 360

    print(wedge_patch.theta1, wedge_patch.theta2)
    print(wedge_patch.center)
    return circle_patch, wedge_patch


"""
Begins the animation
"""
anim = animation.FuncAnimation(
    fig, animate, init_func=init, frames=360, interval=20, blit=True
)

plt.show()
