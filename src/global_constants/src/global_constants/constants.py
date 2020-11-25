#!/usr/bin/env python

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

import numpy as np

from utils import rpy_to_quaternion


x, y = 2, 0
CAMERA_DATA = [
    ["camera0", (x, y, 1), (0, np.pi / 2, 0)],
    ["camera1", (x - 1, y, 0), (0, 0, 0)],
    ["camera2", (2.3, -.75, 1), (.2, .7, 1.55)],
    ["camera3", (3, -.5, .5), (0, .2, 2.3)],
]
CAMERAS = {d[0]: Pose(position=Point(*d[1]), orientation=Quaternion(*rpy_to_quaternion(*d[2]))) for d in CAMERA_DATA}
