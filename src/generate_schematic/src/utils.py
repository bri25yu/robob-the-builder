import numpy as np


def unify_colors(img):
    """
    Produce a new image where shadows appear same color as block_size

    Parameters
    ----------
    img : ndarray

    Returns
    ---------
    uniform_img : ndarray
    """
    sums = np.reshape(np.sum(img, axis = 2), (img.shape[0], img.shape[1], 1)).astype(np.float32)
    sums[sums == 0] = 0.01
    return (img/sums * 255).astype(np.uint8)
