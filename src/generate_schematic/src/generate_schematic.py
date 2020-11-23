#!/usr/bin/env python

import numpy as np

import os

import cv2

import constants as const

import matplotlib.pyplot as plt



IMAGE_IN_PATH = "images/gazebo_angled_image_one.jpg"
IMAGE_OUT_NAME = "gazebo_angled_image_one_clustering.jpg"
def main():
    g = GenerateSchematic()
    img = g.get_image(IMAGE_IN_PATH)
    bottom_left_coordinates = g.find_all_bottom_left_coordinates_2d(img)

    #display an image with bottom left coordinates highlighted in white
    for coordinates in bottom_left_coordinates:
        cv2.circle(img, coordinates, 3, (255, 255, 255), -1)
    cv2.imshow("BottomLeftCoordinates", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

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
            The image to perform segmentation on.

        Returns
        -------
        edge_detected : np.ndarray
            The input image with segmentation performed.

        """
        if segmentation_method is None:
            segmentation_method = Segmentation.edge_detect_canny

        return segmentation_method(img, **kwargs)

    def find_image_bottom_left_coordinates_2d(self, img):
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
        #convert to grayscale and threshold so blocks appear white
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 15, 255, cv2.THRESH_BINARY)

        #find contours of blocks
        _, contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #loop through contours and find bottom left coordinates
        results = []
        for contour in contours:
            #note, min_x and max_x are in image coordinates (so x increases to the right and y increases downwards)
            min_x = np.min(contour[:, 0, 0])
            max_y = np.max(contour[:, 0, 1])
            results.append((min_x, max_y))
        return results

    def find_all_bottom_left_coordinates_2d(self, img):
        """
        Parameters
        ----------
        img : np.ndarray
            Image with blocks to find bottom left block coordinates from.
            The blocks may be different colors.

        Returns
        -------
        results : np.ndarray
            An array of bottom left block coordinates in the image
        """
        #perform clustering to divide image into groups of the same color
        NUM_CLUSTERS = 11
        segmented, clustered_segments, labels_bincount = self.segment(img, segmentation_method=Segmentation.cluster_segment, n_clusters=NUM_CLUSTERS)
        for i, segment in enumerate(clustered_segments):
            #save each cluster to a separate image
            self.save_image(segment, IMAGE_OUT_NAME + "_" + str(i) + ".jpg")
        #labels_bincount represents the number of pixels in each cluster
        total_labels = sum(labels_bincount)
        bottom_left_coordinates = []
        for i in range(NUM_CLUSTERS):
            percent_data = labels_bincount[i]/float(total_labels)
            #if this is a cluster we want to look at
            #(has percent_data within a certain range, indicating that the cluster has boxes)
            if percent_data > .001 and percent_data < .5:
                #read image and find its bottom left block coordinates
                path_to_image = "images/" + IMAGE_OUT_NAME + "_" + str(i) + ".jpg"
                image_bottom_left_coordinates = self.find_image_bottom_left_coordinates_2d(self.get_image(path_to_image))
                bottom_left_coordinates.extend(image_bottom_left_coordinates)
        return bottom_left_coordinates

    def unify_colors(self, img):
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
        return (img/sums * 255).astype(np.uint8)


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
        clusters = np.uint8(clusters)
        intermediateImage = clusters[labels.flatten()]
        clusteredImage = intermediateImage.reshape((originalImage.shape))

        clusteredSegments = []
        for i in range(n_clusters):
            new_image = reshapedImage * (labels == i)
            new_image = new_image.reshape((originalImage.shape))
            clusteredSegments.append(new_image)
        return clusteredImage, clusteredSegments, np.bincount(labels.flatten())

    #----------------------------------------------------------------------------------------------
    # Helpers



if __name__ == "__main__":
    main()
