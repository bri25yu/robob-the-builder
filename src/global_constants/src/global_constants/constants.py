#!/usr/bin/env python

import numpy as np


IMG_DIR = "images"


x, y = 2, 0
CAMERA_DATA = [
    [(x, y, 1), (0, np.pi / 2, 0)],
    [(x - 1, y, 0), (0, 0, 0)],
    [(2.3, -.75, 1), (.2, .7, 1.55)],
    [(2.4, -.75, 1.1), (.2, .7, 1.55)],
]

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
