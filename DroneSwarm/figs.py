import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

ruh_m = np.zeros((500,500,3), np.uint8)

BBox = (-.001, .001, -.001, .001)
fig, ax = plt.subplots(figsize = (8,7))

ax.scatter(float(7.845758093769428e-06), float(0.00013565055062412357), zorder=1, alpha= 0.2, c='#ff43a4', s=10)
ax.scatter(float(7.557433224519353e-05), float(0.000328921011268967), zorder=1, alpha= 0.2, c='#ff43a4', s=10)
ax.scatter(float(8.836376238035083e-05), float(0.0005587768609407589), zorder=1, alpha= 0.2, c='#ff43a4', s=10)

ax.set_title('Plotting Spatial Map')
ax.set_xlim(BBox[0],BBox[1])
ax.set_ylim(BBox[2],BBox[3])
ax.imshow(ruh_m, zorder=0, extent = BBox, aspect= 'equal')
fig.savefig('C:/Users/marii/source/repos/AirSim/PythonClient/multirotor/Drone-Search-and-Rescue-SD-2/DroneSwarm/filename.png')