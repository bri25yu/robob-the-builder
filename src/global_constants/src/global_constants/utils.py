import os

import numpy as np
import cv2

import constants as const


def rpy_to_quaternion(roll, pitch, yaw):
    """
    Parameters
    ----------
    roll: float
    pitch: float
    yaw: float

    Returns
    -------
    quaternion: list
        [x, y, z, w]

    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy

    return x, y, z, w


def get_image(path):
    """
    Parameters
    ----------
    path : str
        The path to the image to load.

    Returns
    -------
    img : np.ndarray
        The image located at the input path.

    """
    return cv2.imread(path)


def save_image(img, name):
    """
    Parameters
    ----------
    img : np.ndarray
        Image to save.
    name : str
        Name of the image to save.

    """
    if const.IMG_DIR not in name:
        name = os.path.join(const.IMG_DIR, name)

    cv2.imwrite(name, img)
