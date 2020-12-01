import cv2
import numpy as np

from global_constants import utils as gutils
from segmentation import Segmentation
from image_matching import ImageMatching


orb = cv2.ORB_create()


class FeatureDetect:

    @staticmethod
    def find_all_bottom_left_coordinates_2d(img, num_clusters=11, low=0.001, high=0.5):
        """
        Given an image that is only 2d, find and return all of the bottom left corners of blocks in the image.

        Parameters
        ----------
        img: np.ndarray
            Image with blocks to find bottom left block coordinates from.
            The blocks may be different colors.
        num_clusters: int
            The number of clusters to cluster around.

        Returns
        -------
        bottom_left_coordinates: np.ndarray
            An array of bottom left block coordinates in the image

        """
        # Perform clustering to divide image into groups of the same color
        clustered_segments, labels_bincount = Segmentation.cluster_segment(img, n_clusters=num_clusters)

        # labels_bincount represents the number of pixels in each cluster
        total_labels = sum(labels_bincount)
        bottom_left_coordinates = []
        for i in range(num_clusters):
            percent_data = labels_bincount[i]/float(total_labels)
            # If this is a cluster we want to look at i.e. it has percent_data within a certain range,
            # indicating that the cluster has boxes but is not a background
            if low < percent_data < high:
                # Find its bottom left block coordinates
                image_bottom_left_coordinates = BottomLeft.find_image_bottom_left_coordinates_2d(clustered_segments[i])
                bottom_left_coordinates.extend(image_bottom_left_coordinates)

        return bottom_left_coordinates

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

    # Helpers--------------------------------------------------------------------------------------

    @staticmethod
    def find_image_bottom_left_coordinates_2d(img):
        """
        Parameters
        ----------
        img : np.ndarray
            Image with blocks to find bottom left block coordinates from.
            The blocks in the image should all be the same color.

        Returns
        -------
        results : np.ndarray
            An array of bottom left block coordinates in the image

        """
        # Convert to grayscale and threshold so blocks appear white
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 15, 255, cv2.THRESH_BINARY)

        # Find contours of blocks
        _, contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Loop through contours and find bottom left coordinates
        results = []
        for contour in contours:
            # NOTE, min_x and max_x are in image coordinates (so x increases to the right and y increases downwards)
            min_x = np.min(contour[:, 0, 0])
            max_y = np.max(contour[:, 0, 1])
            results.append((min_x, max_y))
        return results
