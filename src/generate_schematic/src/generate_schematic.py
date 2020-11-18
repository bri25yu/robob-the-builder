#!/usr/bin/env python

import numpy as np

import os

import cv2

import constants as const


IMAGE_IN_PATH = "images/gazebo_image_two.jpg"
IMAGE_OUT_NAME = "gazebo_image_two_clustering.jpg"
def main():
    g = GenerateSchematic()

    img = g.get_image(IMAGE_IN_PATH)
    NUM_CLUSTERS = 11
    segmented, clustered_segments = g.segment(img, segmentation_method=Segmentation.cluster_segment, n_clusters=NUM_CLUSTERS)
    for i, segment in enumerate(clustered_segments):
        g.save_image(segment, IMAGE_OUT_NAME + "_" + str(i) + ".jpg")
    # segmented = (segmented * (255 / (NUM_CLUSTERS-1))).astype(np.uint8)
    # segmented = g.segment(img, segmentation_method=Segmentation.edge_detect_canny)
    g.save_image(segmented, IMAGE_OUT_NAME)


class GenerateSchematic:

    def get_image(self, path):
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

    def save_image(self, img, name):
        """
        Parameters
        ----------
        img : np.ndarray
            Image to save.
        name : str
            Name of the image to save.

        """
        cv2.imwrite(os.path.join(const.IMG_DIR, name), img)

    def segment(self, img, segmentation_method=None, **kwargs):
        """
        Parameters
        ----------
        img : np.ndarray
            The image to perform edge detection on.

        Returns
        -------
        edge_detected : np.ndarray
            The input image with edge detection performed.

        """
        if segmentation_method is None:
            segmentation_method = Segmentation.edge_detect_canny

        return segmentation_method(img, **kwargs)


class Segmentation:

    @staticmethod
    def edge_detect_canny(img, min_val=100, max_val=200):
        """
        Perform Canny edge detection.
        See https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_canny/py_canny.html

        Parameters
        ----------
        gray_img : ndarray
            grayscale image array

        Returns
        -------
        ndarray
            gray_img with edges outlined
        """
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sigma = .33
        v = np.median(gray_img)

        #---- apply automatic Canny edge detection using the computed median----
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edges = cv2.Canny(gray_img, lower, upper)
        return edges

    @staticmethod
    def cluster_segment(img, n_clusters=3, random_state=0):
        """segment image using k_means clustering

        Parameter
        ---------
        img : ndarray
            rgb image array
        n_clusters : int
            the number of clusters to form as well as the number of centroids to generate
        random_state : int
            determines random number generation for centroid initialization

        Returns
        -------
        ndarray
            clusters of gray_img represented with similar pixel values
        """
        originalImage = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        reshapedImage = np.float32(originalImage.reshape(-1, 3))
        stopCriteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.1)
        ret, labels, clusters = cv2.kmeans(reshapedImage, n_clusters, None, stopCriteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        print("labels", np.bincount(labels.flatten()))
        print("ret", ret)
        print("clusters", clusters.shape)
        clusters = np.uint8(clusters)
        intermediateImage = clusters[labels.flatten()]
        clusteredImage = intermediateImage.reshape((originalImage.shape))

        clusteredSegments = []
        for i in range(n_clusters):
            new_image = reshapedImage * (labels == i)
            new_image = new_image.reshape((originalImage.shape))
            clusteredSegments.append(new_image)
        return clusteredImage, clusteredSegments
        # Downsample img by a factor of 2 first using the mean to speed up K-means
        # img_d = cv2.resize(img, dsize=(img.shape[1]/2, img.shape[0]/2), interpolation=cv2.INTER_NEAREST)
        # # img_d = img
        #
        # # first convert our 3-dimensional img_d array to a 2-dimensional array
        # # whose shape will be (length * width, number of channels) hint: use img_d.shape
        # img_r = np.reshape(img_d, (img_d.shape[0] * img_d.shape[1], img_d.shape[2]))
        #
        # # Fit the k-means algorithm on this reshaped array img_r using the the do_kmeans function defined above.
        # clusters = Segmentation._do_kmeans(img_r, n_clusters)
        # print(clusters.shape)
        # # reshape this clustered image to the original downsampled image (img_d) shape
        # cluster_img = np.reshape(img_r, img_d.shape)
        #
        # # Upsample the image back to the original image (img) using nearest interpolation
        # img_u = cv2.resize(src=cluster_img, dsize=(img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
        # # img_u = cluster_img
        #
        # # return img_u.astype(np.uint8)
        # return cluster_img

    #----------------------------------------------------------------------------------------------
    # Helpers

    @staticmethod
    def _do_kmeans(data, n_clusters):
        """Uses opencv to perform k-means clustering on the data given. Clusters it into
        n_clusters clusters.

        Args:
            data: ndarray of shape (n_datapoints, dim)
            n_clusters: int, number of clusters to divide into.

        Returns:
            clusters: integer array of length n_datapoints. clusters[i] is
            a number in range(n_clusters) specifying which cluster data[i]
            was assigned to.
        """
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2)
        _, clusters, centers = kmeans = cv2.kmeans(data.astype(np.float32), n_clusters, bestLabels=None, criteria=criteria, attempts=1, flags=cv2.KMEANS_RANDOM_CENTERS)
        print(centers)
        return clusters


if __name__ == "__main__":
    main()
