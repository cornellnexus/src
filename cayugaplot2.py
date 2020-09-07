import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import time
from Graph import Graph

# read a file, and plot the points in the file one by one

BoundaryBox = [-76.5119, -76.5013, 42.4596, 42.4642]
ruh_m = plt.imread('map.png')
fig, ax = plt.subplots(figsize=(8, 7))

ax.set_title('Cayuga Lake Shore')
ax.set_xlim(BoundaryBox[0], BoundaryBox[1])
ax.set_ylim(BoundaryBox[2], BoundaryBox[3])
ax.imshow(ruh_m, zorder=0, extent=BoundaryBox, aspect='equal')


def get_longs(coord_list):
    longs = []
    for coord in coord_list:
        longs.append(coord[0])
    return longs


def get_lats(coords):
    lats = []
    for coord in coords:
        lats.append(coords[1])
    return lats


def animate(i):
    #df = pd.read_csv("cayugacoords.txt")

    graph = Graph(-76.5119, -76.5013, 42.4596, 42.4642)
    # all coords traversed
    coordsList = graph.startDFSTraversalAtCoordinate(42.4596, -76.5119)

    xcoords = get_longs

    points = ax.scatter(
        df.longitude[:i+1], df.latitude[:i+1], zorder=1, alpha=1, c='r', s=10)
    return


def update(event):
    print("clicked start")
    anim = animation.FuncAnimation(fig, animate, repeat=False, interval=250)
    plt.show()
    return


updatePos = plt.axes([0.5, 0.0, 0.3, 0.075])
bUpdate = Button(updatePos, 'Plot points in file', color='green')
bUpdate.on_clicked(update)

plt.show()
