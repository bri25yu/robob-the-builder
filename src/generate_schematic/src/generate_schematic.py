#!/usr/bin/env python

import numpy as np

import os

import cv2

import constants as const


IMAGE_IN_PATH = "images/gazebo_image_two.jpg"
IMAGE_OUT_NAME = "gazebo_image_two_segmented.jpg"
def main():
    g = GenerateSchematic()

    img = g.get_image(IMAGE_IN_PATH)
    segemented = g.segment(img, min_val=1, max_val=254)
    g.save_image(segemented, IMAGE_OUT_NAME)


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
            segmentation_method = self.edge_detect_canny

        return segmentation_method(img, **kwargs)

    #----------------------------------------------------------------------------------------------
    # Helpers

    def edge_detect_canny(self, img, min_val=100, max_val=200):
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
        edges = cv2.Canny(gray_img, min_val, max_val)

        return edges


if __name__ == "__main__":
    main()
