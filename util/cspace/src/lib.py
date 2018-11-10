from shapely import speedups
from shapely.geometry import *
from shapely.vectorized import *
if speedups.available:
    speedups.enable()

import numpy as np
import matplotlib
import yaml

def createObstacles(data: dict) -> [Polygon]:
    obstacleDict: [Polygon] = []
    for key in ['robot', 'ground']:
        points: [float] = []
        for vertex in data[key]['vertices']:
            points.append([vertex['x'], vertex['y']])
        obstacleDict.append(Polygon(points))
        print(points)
    return obstacleDict
