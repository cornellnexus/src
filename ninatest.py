import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button

#df = pd.read_csv("cayugacoords.txt")

BoundaryBox = [-76.5119, -76.5013, 42.4596, 42.4642]
ruh_m = plt.imread('map.png')
fig, ax = plt.subplots(figsize=(8, 7))
#ax = plt.subplot(111)

ax.set_title('Cayuga Lake Shore')
ax.set_xlim(BoundaryBox[0], BoundaryBox[1])
ax.set_ylim(BoundaryBox[2], BoundaryBox[3])
ax.imshow(ruh_m, zorder=0, extent=BoundaryBox, aspect='equal')


def update(event):
    print("cornell")
    df = pd.read_csv("cayugacoords.txt")
    points = ax.scatter(df.longitude, df.latitude, zorder=1,
                        alpha=1, c='r', s=10)
    plt.draw()


axcut = plt.axes([0.9, 0.0, 0.1, 0.075])
bUpdate = Button(axcut, 'Update', color='red', hovercolor='green')
bUpdate.on_clicked(update)

# def animate(nframe):
#     ax.collections = []
#     df = pd.read_csv("cayugacoords.txt")
#     points = ax.scatter(df.longitude, df.latitude, zorder=1,
#                         alpha=1, c='r', s=10)
#     return

# anim = animation.FuncAnimation(fig, animate)
plt.show()
