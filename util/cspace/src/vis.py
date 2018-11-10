
from calculateCSpace import forwardKinematics

import sys
from calculateCSpace import forwardKinematics, isIntersecting, obstacles
from shapely.geometry import *
import shapely
import shapely.geometry
import cv2
import numpy as np

def nothing(x):
    pass

cv2.namedWindow('image')
cv2.createTrackbar('theta','image',-35,175,nothing)
cv2.createTrackbar('phi','image',-180,180,nothing)
cv2.setTrackbarMin('theta', 'image', -35)
cv2.setTrackbarMin('phi', 'image', -180)

elbowPos = np.asarray([0.0, 0.0])

def ro(tup):
    return (int(tup[0]) + 256, 512 - (int(tup[1]) + 256))

while True:
    theta = cv2.getTrackbarPos('theta', 'image')
    phi = cv2.getTrackbarPos('phi', 'image')
    clawPos, wristPos = forwardKinematics(theta, phi, False)

    clawPos = np.asarray(clawPos)
    wristPos = np.asarray(wristPos)

    ob1: Polygon = obstacles[0]
    ob2: Polygon = obstacles[1]

    x1, y1 = ob1.exterior.coords.xy
    x2, y2 = ob2.exterior.coords.xy

    mul = 5

    img = np.ones((512,512,3))

    for i in range(len(x1)-1):
        point1 = ro((x1[i]*mul, y1[i]*mul))
        point2 = ro((x1[i+1]*mul, y1[i+1]*mul))
        cv2.line(img, point1, point2, (0, 255, 255))

    for i in range(len(x2)-1):
        point1 = ro((x2[i]*mul, y2[i]*mul))
        point2 = ro((x2[i+1]*mul, y2[i+1]*mul))
        cv2.line(img, point1, point2, (255, 255, 0))

    cv2.circle(img, ro(clawPos*mul), 5, (255, 0, 0), -1)
    cv2.circle(img, ro(wristPos*mul), 5, (255, 0, 0), -1)
    cv2.circle(img, ro(elbowPos), 5, (255, 0, 0), -1)

    cv2.imshow('image', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
