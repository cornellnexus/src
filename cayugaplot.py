import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("cayugacoords.txt")

# BBox = ((df.longitude.min(),   df.longitude.max(),
#          df.latitude.min(), df.latitude.max()))
# #         > (42.4596, 42.4642, -76.5119, -76.5013)

BBox = [-76.5119, -76.5013, 42.4596, 42.4642]

ruh_m = plt.imread('map.png')

fig, ax = plt.subplots(figsize=(8, 7))
ax.scatter(df.longitude, df.latitude, zorder=1, alpha=1, c='r', s=10)
ax.set_title('Cayuga Lake Shore')
ax.set_xlim(BBox[0], BBox[1])
ax.set_ylim(BBox[2], BBox[3])


ax.imshow(ruh_m, zorder=0, extent=BBox, aspect='equal')

plt.show()
