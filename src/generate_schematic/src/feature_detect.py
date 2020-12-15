import cv2
import numpy as np

from global_constants import utils as gutils
from image_matching import ImageMatching


orb = cv2.ORB_create()


class FeatureDetect:

    @staticmethod
    def find_all_corners_3d(camera1, camera2, epipolar_threshold=0.11):
        """
        Parameters
        ----------
        camera1: CameraDTO
        camera2: CameraDTO
        epipolar_threshold: float

        Returns
        -------
        corners: list
            A list of all of the corner positions that can be recovered from the two input images.

        """
        kp1, des1 = FeatureDetect.get_corners_harris(camera1.image)
        kp2, des2 = FeatureDetect.get_corners_harris(camera2.image)
        return ImageMatching.match_2_cameras_images(camera1, camera2, kp1, des1, kp2, des2, epipolar_threshold=epipolar_threshold)

    @staticmethod
    def get_corners_harris(img, blocksize=5, sobel_constant=7, threshold=0.01):
        indices = []
        for i in range(img.shape[2]):
            dst = cv2.cornerHarris(img[:, :, i], blocksize, sobel_constant, threshold)
            potential_indices = np.argwhere(dst > (0.01 * dst.max()))
            if len(potential_indices) > 0:
                indices.append(potential_indices)
        indices = np.vstack(indices)
        indices = np.hstack((indices[:, 1:2], indices[:, 0:1]))

        kp = np.hstack((indices, np.ones((len(indices), 1)) * blocksize))
        kp = [cv2.KeyPoint(*p) for p in kp]

        kp, des = orb.compute(img, kp)

        return kp, des
