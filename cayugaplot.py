import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
# Update plot as a file is changed

#df = pd.read_csv("cayugacoords.txt")
BoundaryBox = [-76.5119, -76.5013, 42.4596, 42.4642]
ruh_m = plt.imread('map.png')
fig, ax = plt.subplots(figsize=(8, 7))

ax.set_title('Cayuga Lake Shore')
ax.set_xlim(BoundaryBox[0], BoundaryBox[1])
ax.set_ylim(BoundaryBox[2], BoundaryBox[3])
ax.imshow(ruh_m, zorder=0, extent=BoundaryBox, aspect='equal')


def animate(nframe):
    ax.collections = []
    df = pd.read_csv("cayugacoords.txt")
    points = ax.scatter(df.longitude, df.latitude, zorder=1,
                        alpha=1, c='r', s=10)
    return


anim = animation.FuncAnimation(fig, animate)
plt.show()
