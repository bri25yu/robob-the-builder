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
        kp1, des1 = orb.detectAndCompute(camera1.image, None)
        kp2, des2 = orb.detectAndCompute(camera2.image, None)
        return ImageMatching.match_2_cameras_images(camera1, camera2, kp1, des1, kp2, des2, epipolar_threshold=epipolar_threshold)
