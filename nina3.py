import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import imageio

BBox = [-76.5119, -76.5013, 42.4596, 42.4642]

img = imageio.imread('map.png')

fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(7, 6.5)

#ax = plt.axes(xlim=(0, 20), ylim=(0, 20))
ax = plt.axes(xlim=(BBox[0], BBox[1]), ylim=(BBox[2], BBox[3]))
#patch = plt.Circle((5, -5), 0.75, fc='y')
# 0.0001 is the smallest radius I can get
patch = plt.Circle((-76.51, 42.46), 0.0001, fc='y')


def init():
    #patch.center = (5, 5)
    patch.center = (-76.51, 42.46)
    ax.add_patch(patch)
    return patch,


def animate(i):
    pullData = open("ninacoords.txt", "r").read()
    dataArray = pullData.split('\n')
    xar = []
    yar = []
    for eachLine in dataArray:
        if len(eachLine) > 1:
            x, y = eachLine.split(',')
            xar.append(float(x))
            yar.append(float(y))
    #x = 10 + 3 * np.sin(np.radians(i))
    #y = 10 + 3 * np.cos(np.radians(i))
    #patch.center = (x, y)
    patch.center = (xar[-1], yar[-1])
    return patch,


anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=360,
                               interval=20,
                               blit=True)

plt.imshow(img, zorder=0,  extent=BBox)
plt.show()
