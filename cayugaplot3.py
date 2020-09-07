import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
from Graph import Graph
from UserUtils import *

# generating DFS then plotting the points one by one WITHOUT using a file
# 9/7

#df = pd.read_csv("cayugacoords.txt")
#BoundaryBox = [-76.5119, -76.5013, 42.4596, 42.4642]

longMin, longMax, latMin, latMax = getLongLatMinMaxFromUser()

BoundaryBox = [longMin, longMax, latMin, latMax]
ruh_m = plt.imread('map.png')
fig, ax = plt.subplots(figsize=(8, 7))

ax.set_title('Cayuga Lake Shore')
ax.set_xlim(BoundaryBox[0], BoundaryBox[1])
ax.set_ylim(BoundaryBox[2], BoundaryBox[3])
ax.imshow(ruh_m, zorder=0, extent=BoundaryBox, aspect='equal')

graph = Graph(longMin, longMax, latMin, latMax)
# all coords traversed
coordsList = graph.startDFSTraversalAtCoordinate(latMin, longMin)
print("coordlist line 28")
print(coordsList)

# engine = Engine()
# coordsList = engine.run()
# print("coordlist line 28")
# print(coordsList)


def get_longs(coord_list):
    longs = []
    for coord in coord_list:
        longs.append(coord[1])
    return longs


def get_lats(coords):
    lats = []
    for coord in coords:
        lats.append(coord[0])
    return lats


xcoords = np.array(get_longs(coordsList))
ycoords = np.array(get_lats(coordsList))


def animate(i):
    print("here")
    # print(coordsList)
    # print(xcoords)
    # print(ycoords)
    points = ax.scatter(xcoords[:i+1], ycoords[:i+1],
                        zorder=1, alpha=1, c='r', s=10)
    return


def update(event):
    print("clicked")

    # engine = Engine()
    # coordsList = engine.run()
    # #df = pd.read_csv("cayugacoords.txt")
    # points = ax.scatter(df.longitude, df.latitude, zorder=1,
    #                     alpha=1, c='r', s=10)
    # plt.draw()

    anim = animation.FuncAnimation(fig, animate, repeat=False, interval=250)
    plt.show()
    return


axcut = plt.axes([0.8, 0.0, 0.2, 0.075])
bUpdate = Button(axcut, 'Plot DFS', color='red', hovercolor='green')
bUpdate.on_clicked(update)

# def animate(nframe):
#     ax.collections = []
#     df = pd.read_csv("cayugacoords.txt")
#     points = ax.scatter(df.longitude, df.latitude, zorder=1,
#                         alpha=1, c='r', s=10)
#     return

# anim = animation.FuncAnimation(fig, animate)
plt.show()
