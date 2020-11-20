import numpy as np


def get_content_between(s, start_flag, end_flag):
    """
    Returns the substring from the first instance of the input start_flag
    to the first instance of end_flag.

    Parameters
    ----------
    s: str
    start_flag: str
    end_flag: str

    """
    start_i = s.find(start_flag)
    end_i = s.find(end_flag, start_i)
    return s[start_i + len(start_flag): end_i]


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
