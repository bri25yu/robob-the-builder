import os
import pickle

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


def unique_rows(a):
    a = np.ascontiguousarray(a)
    unique_a = np.unique(a.view([('', a.dtype)]*a.shape[1]))
    return unique_a.view(a.dtype).reshape((unique_a.shape[0], a.shape[1]))


def round_nearest(number, offset, multiple):
    return np.round((number - offset) / multiple) * multiple + offset


def close_to_multiples_of(coordinates, multiple, offset, tolerance=.05):
    dists = np.abs(((coordinates - offset) % multiple) - (multiple / 2)) - (multiple / 2)
    close = np.ones(len(coordinates))
    for i in range(coordinates.shape[1]):
        close = close * np.isclose(dists[:, i], 0, atol=tolerance[i])
    return np.ravel(np.argwhere(close))


def get_layers(coordinates):
    heights = np.unique(coordinates[:, 2])
    layers = []
    for height in heights:
        layer = unique_rows(coordinates[np.isclose(coordinates[:, 2], height)])
        layer = layer[np.argsort(-layer[:, 1])]
        layer = layer[np.argsort(-layer[:, 0])]
        layers.append(layer)
    return np.array(layers)


def get_bottom_left_corners(coordinates):
    """
    Bottom left is defined as the min(x), min(y), min(z).d

    Parameters
    ----------
    coordinates: (n, 3)-shaped np.ndarray

    Returns
    -------
    bottom_left_corners: np.ndarray

    """
    max_x, max_y = np.max(coordinates[:, 0]), np.max(coordinates[:, 1])
    return coordinates[np.ravel(np.argwhere(1 - (np.isclose(coordinates[:, 0], max_x) | np.isclose(coordinates[:, 1], max_y))))]


def output_corners(layers, filename=const.CORNERS_OUTPUT_FILE):
    SPACING = 2
    #only include coordinates with z coordinate 0 and add spacing
    layers = np.array([c for c in layers if c[2] == 0])
    x_median = np.median(layers[:, 0])
    y_median = np.median(layers[:, 1])
    layers = np.array([[(c[0] - x_median) * SPACING + x_median, (c[1] - y_median) * SPACING + y_median, 0] for c in layers])
    print(layers)
    with open(filename, "wb") as file:
        pickle.dump(layers, file)


def get_corners(filename=const.CORNERS_OUTPUT_FILE):
    with open(filename, "rb") as file:
        return get_layers(pickle.load(file))
