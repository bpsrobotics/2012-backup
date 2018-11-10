#!/usr/bin/env python3


# This script uses brute force to translate a cartesian (x, y) vector occupancy
# map into a bipolar (θ, φ) map, also known as c-space (configuration space).

# Traditionally, you transform obstacles and lines from cartesian space
# into c-space using inverse kinematics, computing the arm poses needed to hit
# the various defined obstacles. However, we're needing to A. move at high
# speeds (rio no likey inverse kinematics) and B. deal with the fact our arm is
# double sided and C. really really don't want to write our own IK calculations
# since A. we're going to be defining target wrist+arm pose instead of claw tip
# position and B. that's a pain in the ass. So, that means that traditional
# c-space transformation is off the table. Instead, we're taking advantage of
# the fact that we don't need to run any transforms in real time on the rio,
# and simply brute forcing it. We define the polytope area in cartesian space
# that touching would be a Bad Thing (floor, back of robot), and then we define
# the arm angle limits and characteristics (length and stuff). With that data,
# we use a monte-carlo approach to transform our obstacles from cartesian to
# c-space. Since the wrist has two protrusions (claw front and back), we have
# to do this twice and then merge the results. What this means is that we
# sample a shitton of points in c-space (like >250^2), then transform those
# points into cartesian space using forward kinematics, then mark down whether
# or not that pose will intersect with something we really don't want to
# intersect with. After that, we'll run it again for the other side of the
# effector and add the found colliding point sets together. From there, we
# should be able to find a rough contour and then a convex hull for each cluster
# of points, then create a vector occupancy map that we can load onto the robot
# for use in path (pose?) planning.


# Let's break it down into steps so we can document well

# Step 1: Define arm characteristics
# Step 2: Define cartesian vector occupancy map as a collection of polytopes
# Step 3: Devise forward kinematic equation
# Step 4: Find intersecting points for the first effector
# Step 5: Find intersecting points for the second effector
# Step 6: Combine point sets
# Step 7: Find c-space vector occupancy map as a collection of polytopes
# Step 7.5: Save unified c-space point set for visualization and reference
# Step 8: Transform c-space map into cartesian space and validate/sanity check
# Step 9: Export into a format for use in the robot (json?) and go take a nap
from multiprocessing.pool import Pool
from threading import Thread

import progressbar
from multiprocessing import Lock

from os.path import exists
from shapely import speedups

if speedups.available:
    speedups.enable()
from descartes import *

from shapely.geometry import *
from shapely.vectorized import *

import numpy as np
import matplotlib
from matplotlib import pyplot
import yaml
import math
import operator
import os.path

from lib import *

with open('characteristics.yaml', 'r') as stream:
    loadedYaml: dict = yaml.load(stream, yaml.CLoader)
ly: dict = loadedYaml

DEBUG = False

# Step 1
ARM_LENGTH: float = ly['arm']['length']
ARM_THICKNESS: float = ly['arm']['thickness']
ELBOW_MIN_ANGLE: float = ly['arm']['angle']['min']
ELBOW_MAX_ANGLE: float = ly['arm']['angle']['max']

CLAW_LENGTH: float = ly['wrist']['clawLength']
PISTON_LENGTH: float = ly['wrist']['pistonLength']
WRIST_THICKNESS: float = ly['wrist']['thickness']
WRIST_MIN_ANGLE: float = ly['wrist']['angle']['min']
WRIST_MAX_ANGLE: float = ly['wrist']['angle']['max']

ELBOW_SAMPLES: float = ly['solver']['samples']['theta']
WRIST_SAMPLES: float = ly['solver']['samples']['phi']

OBSTACLE_BUFFER: float = ly['obstacles']['expand']

# Step 2
obstacles: [Polygon] = createObstacles(loadedYaml['obstacles'])

if DEBUG:
    fig = pyplot.figure(1, figsize=(5, 5), dpi=180)
    ax = fig.add_subplot(121)

BLUE = '#6699cc'
GRAY = '#999999'

if DEBUG:
    patch1 = PolygonPatch(obstacles[0], fc=BLUE, ec=BLUE, alpha=0.5, zorder=2)
    patch2 = PolygonPatch(obstacles[1], fc=GRAY, ec=GRAY, alpha=0.5, zorder=2)
    ax.add_patch(patch1)
    ax.add_patch(patch2)

obstacles: [Polygon] = \
    [obstacle.buffer(
        OBSTACLE_BUFFER,
        cap_style=CAP_STYLE.round,
        join_style=JOIN_STYLE.round
    ) for obstacle in obstacles]

if DEBUG:
    patch1 = PolygonPatch(obstacles[0], fc=BLUE, ec=BLUE, alpha=0.5, zorder=2)
    patch2 = PolygonPatch(obstacles[1], fc=GRAY, ec=GRAY, alpha=0.5, zorder=2)
    ax.add_patch(patch1)
    ax.add_patch(patch2)
    pyplot.xlim((-20, 20))
    pyplot.ylim((-13, 20))
    pyplot.show()


# Step 3

def forwardKinematics(theta: float, phi: float, pistonNotClaw=False) -> [float, float]:
    assert ELBOW_MIN_ANGLE <= theta <= ELBOW_MAX_ANGLE
    assert WRIST_MIN_ANGLE <= phi <= WRIST_MAX_ANGLE

    # Elbow joint is at (0, 0)
    # Wrist joint position = [ARM_LENGTH*cos(theta) ARM_LENGTH*sin(theta)]
    wristJointPos: [float, float] = [-ARM_LENGTH * math.cos(math.radians(theta)), ARM_LENGTH * math.sin(math.radians(theta))]

    # Determine
    # Wrist joint position = [ARM_LENGTH*cos(theta) ARM_LENGTH*sin(theta)]
    endRelativePos: [float, float] = [0, 0]
    if not pistonNotClaw:
        endRelativePos = [-CLAW_LENGTH * math.cos(math.radians(phi)), CLAW_LENGTH * math.sin(math.radians(phi))]
    else:
        endRelativePos = [-PISTON_LENGTH * math.cos(math.radians(phi) + math.pi), PISTON_LENGTH * math.sin(math.radians(phi) + math.pi)]

    endRelativePos = [endRelativePos[0] + wristJointPos[0], endRelativePos[1] + wristJointPos[1]]
    #print("Theta: {}, Phi: {}, Wrist joint position: {}, End tip position: {}, End tip: {}".format(theta, phi, wristJointPos, endRelativePos, "piston" if pistonNotClaw else "claw"))

    # [wristJointPos, clawOrPistonTipPos]:
    return [endRelativePos, wristJointPos]


def isIntersecting(theta: float, phi: float, pistonNotClaw=False) -> bool:
    # [wristJointPos, clawOrPistonTipPos] ([x, y])
    fkResult: [float, float] = forwardKinematics(theta, phi, pistonNotClaw)
    for obstacle in obstacles:
        if obstacle.intersects(Point(fkResult[0])):
            return False
    return True

if __name__ == '__main__':
    pointsToCheck = np.mgrid[
                    ELBOW_MIN_ANGLE:ELBOW_MAX_ANGLE:ELBOW_SAMPLES * 1j,
                    WRIST_MIN_ANGLE:WRIST_MAX_ANGLE:WRIST_SAMPLES * 1j
                    ]

    x, y = pointsToCheck
    print(x.shape)
    print(y.shape)
    clawPoints: np.ndarray = np.zeros(x.shape)
    pistonPoints: np.ndarray = np.zeros(y.shape)

    totalOperations = (x.shape[0] * x.shape[1])

    bar = progressbar.ProgressBar(redirect_stdout=True, max_value=x.shape[0], widgets=[
        ' [', progressbar.Timer(), '] ',
        progressbar.Bar(),
        ' (', progressbar.ETA(), ') '
    ])



    counter = 0

    # def testRange(num: int, pistonNotClaw=False):
    #     globalCont.counter += 1
    #     bar.update(globalCont.counter)
    #     return np.asarray([[num, j, 1 if isIntersecting(x[num][j], y[num][j], pistonNotClaw) else 0] for j in range(x.shape[1])])

    # p = Pool(4)

    #results = p.map(testRange, [l for l in range(x.shape[0])])

    pistonResults = []
    clawResults = []

    import pickle as pkl
    if os.path.exists('pistonResults.pkl') and os.path.exists('clawResults.pkl'):
        print("Found existing cspace maps, loading...")
        with open('pistonResults.pkl', 'rb') as file:
            pistonResults = pkl.load(file)
        with open('clawResults.pkl', 'rb') as file:
            clawResults = pkl.load(file)
    else:
        print("Didn't find existing cspace maps, generating...")
        for i in range(x.shape[0]):
            for j in range(x.shape[1]):
                pistonResults.append([
                    i,
                    j,
                    isIntersecting(x[i][j], y[i][j], True)
                ])
                clawResults.append([
                    i,
                    j,
                    isIntersecting(x[i][j], y[i][j], False)
                ])
            counter += 1
            bar.update(counter)

        pistonResults = np.asarray(pistonResults)
        clawResults = np.asarray(clawResults)
        with open('pistonResults.pkl', 'wb') as file:
            pkl.dump(pistonResults, file)
        with open('clawResults.pkl', 'wb') as file:
            pkl.dump(clawResults, file)


    bar.finish()

    with open('piston.csv', 'w') as f:
        csv = ""
        for i in pistonResults:
            csv += "{},{},{}\n".format(x[i[0]][i[1]], y[i[0]][i[1]], i[2])
        f.write(csv)

    with open('claw.csv', 'w') as f:
        csv = ""
        for i in clawResults:
            csv += "{},{},{}\n".format(x[i[0]][i[1]], y[i[0]][i[1]], i[2])
        f.write(csv)

    import cv2
    pistonFrame = np.ones((1024, 1024, 1))
    clawFrame = np.ones((1024, 1024, 1))
    #frame = np.ones((1024, 1024, 1))

    total = len(clawResults) + len(pistonResults)

    counter = 0

    print("\n")
    for i in range(len(clawResults)):
        counter += 1
        print("                            ", end='\r')
        print("{} of {}".format(counter, total), end='\r')
        #print(clawResults[i][0], clawResults[i][1], "=",clawResults[i][2])
        clawFrame[clawResults[i][0]][clawResults[i][1]] = 0 if clawResults[i][2] else 1
        #cv2.circle(frame, (clawResults[i][0], clawResults[i][1]),1,(0, 255, 0), -1)

    for i in range(len(pistonResults)):
        counter += 1
        print("                            ", end='\r')
    print("{} of {}".format(counter, total), end='\r')
    #print(pistonResults[i][0], pistonResults[i][1], "=",pistonResults[i][2])
    pistonFrame[pistonResults[i][0]][pistonResults[i][1]] = 0 if pistonResults[i][2] else 1
    #cv2.circle(frame, (pistonResults[i][0], pistonResults[i][1]),1,(0, 255, 0), -1)

    print("\n\n")

    frame = np.logical_or(pistonFrame, clawFrame)

    pistonFrame *= 255
    clawFrame *= 255
    frame = frame * 255

    cv2.imwrite("piston.png", pistonFrame)
    cv2.imwrite("claw.png", clawFrame)
    cv2.imwrite("combined.png", frame)

    import matplotlib.pyplot as plt
    newImage = cv2.imread("combined.png", 1)
    plt.imshow(newImage)
    plt.show()

# for i in range(x.shape[0]):
#     testRange(i)
#     counter += 1
#     print(counter)
#     bar.update(counter)
