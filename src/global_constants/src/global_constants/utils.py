import numpy as np


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
