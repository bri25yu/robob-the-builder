#!/usr/bin/env python

import numpy as np


IMG_DIR = "images"


x, y = 2, 0
CAMERA_DATA = [
    [(x, y, 1), (0, np.pi / 2, 0)],
    [(x - 1, y, 0), (0, 0, 0)],
    # [(x, y, 1), (.2, .7, 1.55)],
    # [(2.3, -.75, 1), (.2, .7, 1.55)],
    # [(2.4, -.75, 1.1), (.2, .7, 1.55)],
    [(2, -.75, .15 - .036), (0, .2, 1.55)],
    [(2.01, -.75, .16 - .036), (0, .2, 1.55)],

]

deltas = [(.3, .3, np.pi + np.pi/4), (-.3, .3, 2 * np.pi -np.pi/4), (-.3, -.3, np.pi/4), (.3, -.3, np.pi/2 + np.pi/4)]
for delta in deltas:
    CAMERA_DATA.append([(x + delta[0], y + delta[1], 1), (0 , 1.15, delta[2])])
    CAMERA_DATA.append([(x + delta[0] + .01, y + delta[1] + .01, 1), (0 , 1.15, delta[2])])

STRUCTURE_TO_BUILD = [  # Passed into WorldSimulation.add_square_2d
    [x, y, 3, 0],
    [x, y, 1, 1],
]

EXPLORATION_BLOCKS = [
    [-3, 3, 0],
    [-1, -4, 0],
    [3, -2, 0],
    [-4, -1, 0],
    [-1,-3, 0],
    [2, 4, 0],
    [1, -4, 0],
    [3, 0, 0],
    [-1, 4, 0],
    [1, -1, 0]
]
