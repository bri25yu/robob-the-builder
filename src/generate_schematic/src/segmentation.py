import cv2
import numpy as np


class Segmentation:

    @staticmethod
    def edge_detect_canny(img, sigma=0.33):
        """
        Perform Canny edge detection.
        See https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_canny/py_canny.html

        Parameters
        ----------
        img: np.ndarray

        Returns
        -------
        edges: np.ndarray
            The edges of the input img. 

        """
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        v = np.median(gray_img)

        #---- apply automatic Canny edge detection using the computed median----
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edges = cv2.Canny(gray_img, lower, upper)
        return edges

    @staticmethod
    def cluster_segment(img, n_clusters=3, random_state=0):
        """
        Segment image using k_means clustering.

        Parameter
        ---------
        img: ndarray
            RGB image array.
        n_clusters: int
            The number of clusters to form as well as the number of centroids to generate.
        random_state: int
            Determines random number generation for centroid initialization.

        Returns
        -------
        clustered_segments: np.ndarray
            An array of images with only a specific cluster displayed.
        label_count: np.ndarray
            An array with counts of labels at each label index. 

        """
        original_shape = img.shape
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        reshapedImage = np.float32(img.reshape(-1, 3))
        stopCriteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.1)
        _, labels, _ = cv2.kmeans(reshapedImage, n_clusters, None, stopCriteria, 10, cv2.KMEANS_RANDOM_CENTERS)

        clustered_segments = []
        for i in range(n_clusters):
            new_image = reshapedImage * (labels == i)
            new_image = new_image.reshape(original_shape)
            clustered_segments.append(new_image)
        return clustered_segments, np.bincount(labels.flatten())
