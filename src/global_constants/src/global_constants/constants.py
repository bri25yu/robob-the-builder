#!/usr/bin/env python

import numpy as np


IMG_DIR = "images"


BLOCK_X = 0.09
BLOCK_Y = 0.09
BLOCK_Z = .355

x, y = 2, 0
CAMERA_DATA = []
zs = [1.3, 2.1]
pitch = 1.15
delta = .325

# CAMERA_DATA = [[(x, y, 2.5), (0, np.pi / 2, 0)], [(x + 0.01, y - 0.01, 2.5), (0, np.pi / 2, 0)]]
# zs = [0.355 * 2]
# pitch = 0
# delta = 1.2

deltas = [(delta, delta, np.pi + np.pi/4), (-delta, delta, 2 * np.pi -np.pi/4), (-delta, -delta, np.pi/4), (delta, -delta, np.pi/2 + np.pi/4)]

for delta in deltas:
    for z in zs:
        CAMERA_DATA.append([(x + delta[0], y + delta[1], z), (0 , pitch, delta[2])])
        CAMERA_DATA.append([(x + delta[0] + .01, y + delta[1] + .01, z), (0 , pitch, delta[2])])

STRUCTURE_TO_BUILD = [  # Passed into WorldSimulation.add_square_2d
    [x, y, 3, 0],
    [x, y, 1, 1],
    [x, y, 1, 2]
    # [x + .01, y + .01, 1, 3]
    # [x, y, 1, 2],
]

EXPLORATION_BLOCKS = [
    #[-3, 3, 0],
    #[-1, -4, 0],
    [3, -2, 0],
    # [-4, -1, 0],
    [-1,-3, 0],
    # [2, 4, 0],
    # [1, -4, 0],
    [3, 0, 0],
    # [-1, 4, 0],
    [1, -1, 0]
]


PICKUP_TRAJECTORY = [
    [0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0],
    [0.21, -1.02, -0.20, 1.94, -1.57, 1.52, 0.0],
    [0.21, 0.35, -0.2, 2.0, -1.57, 1.52, 0.0],
    [0.21, 0.35, -0.2, 0.0, -1.57, 1.52, 0.0],
    [0.21, 0.35, -3.0, 0.0, -1.57, 1.52, 0.0],
    [0.05, -0.07, -3.0, 1.5, -1.57, 0.2, 0.0]
]

PICKUP_TIMES = [0, 1.5, 4.5, 8, 10, 15]

tolerances = np.array([0.01, 0.01, 0.02])
multiple = np.array([BLOCK_X, BLOCK_Y, BLOCK_Z])


KINECT_INTRINSIC_MATRIX_FILE = "output/kinect_intrinsic_matrix.txt"
CORNERS_OUTPUT_FILE = "output/schematic.txt"
