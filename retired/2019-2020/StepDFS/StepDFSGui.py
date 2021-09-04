import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import time
from GraphStepDFS import Graph

dfsRunning = True
started = False

# plt.ion()

BoundaryBox = [-76.5119, -76.5013, 42.4596, 42.4642]
ruh_m = plt.imread('map.png')
fig, ax = plt.subplots(figsize=(8, 7))

plot = ax.scatter([], [])
ax.set_title('Cayuga Lake Shore')
ax.set_xlim(BoundaryBox[0], BoundaryBox[1])
ax.set_ylim(BoundaryBox[2], BoundaryBox[3])
ax.imshow(ruh_m, zorder=0, extent=BoundaryBox, aspect='equal')

# DFS set up
graph = Graph(-76.5119, -76.5013, 42.4596, 42.4642)

xcoords = []
ycoords = []
prevStep = ([], [])


def animate(i):
    points = ax.scatter(xcoords[:i + 1], ycoords[:i + 1], zorder=1,
                        alpha=1, c='r', s=10)

    # points = ax.scatter(df.longitude[:i+1], df.latitude[:i+1], zorder=1,
    #                     alpha=1, c='r', s=10)
    # ax.set_data(df.longitude[:i+1], df.latitude[:i+1])
    return


def update(event):
    counter = 0
    global dfsRunning
    global started
    global xcoords
    global ycoords
    global prevStep

    print("clicked start")

    while (dfsRunning):
        plt.pause(1)

        counter = counter + 1
        if not (started):
            prevStep = graph.startDFSNina(42.4596, -76.5119)
            started = True

            newPoint = prevStep[0][-1]
            xcoords = xcoords + [newPoint[0]]
            ycoords = ycoords + [newPoint[1]]

            # # get the current points as numpy array with shape  (N, 2)
            # array = plot.get_offsets()

            # # add the points to the plot
            # array = np.append(array, (newPoint[0], newPoint[1]))
            # plot.set_offsets(array)

            # # update the figure
            # fig.canvas.draw()

            # # points = ax.scatter(xcoords, ycoords, zorder=1,
            # #                     alpha=1, c='r', s=10)

            plt.scatter(xcoords, ycoords, zorder=5)
            # plt.draw()
        else:
            print(counter)
            # if special indicator in stack
            if (prevStep[1] == [] or prevStep[1][0] == "DONE"):
                dfsRunning = False
                break
            else:
                prevStep = graph.DFSStep(prevStep)

                newPoint = prevStep[0][-1]
                xcoords = xcoords + [newPoint[0]]
                ycoords = ycoords + [newPoint[1]]

                # # get the current points as numpy array with shape  (N, 2)
                # array = plot.get_offsets()
                # # add the points to the plot
                # array = np.append(array, (newPoint[0], newPoint[1]))
                # plot.set_offsets(array)

                # # update the figure
                # fig.canvas.draw()

                # points = ax.scatter(xcoords, ycoords, zorder=1,
                #                     alpha=1, c='r', s=10)
                plt.scatter(xcoords, ycoords, zorder=5)
                # plt.draw()

    # anim = animation.FuncAnimation(fig, animate, repeat=False, interval=250)
    # plt.show()
    return


def stop(event):
    # doesn't do anything yet
    print("clicked stop")
    return


updatePos = plt.axes([0.9, 0.0, 0.1, 0.075])
bUpdate = Button(updatePos, 'Start DFS', color='green')
bUpdate.on_clicked(update)

stopPos = plt.axes([0.8, 0.0, 0.1, 0.075])
bStop = Button(stopPos, 'Stop DFS', color='red')
bStop.on_clicked(stop)

plt.show()
