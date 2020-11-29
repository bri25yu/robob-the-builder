#!/usr/bin/env python

import rospy

from sensor_msgs.msg import CameraInfo

import numpy as np

import os

import cv2

import constants as const

import image_matching as matching

import matplotlib.pyplot as plt

from global_constants import constants as gconst

import tf.transformations as tr


IMAGE_IN_PATH = "images/gazebo_angled_image_one.jpg"
IMAGE_OUT_NAME = "gazebo_angled_image_one_clustering.jpg"

def main():
    rospy.init_node("schematic_node", anonymous = True)
    g = GenerateSchematic()
    g.match_images([2, 3])
    # img = g.get_image(IMAGE_IN_PATH)
    # bottom_left_coordinates = g.find_all_bottom_left_coordinates_2d(img)
    #
    # #display an image with bottom left coordinates highlighted in white
    # for coordinates in bottom_left_coordinates:
    #     cv2.circle(img, coordinates, 3, (255, 255, 255), -1)
    # cv2.imshow("BottomLeftCoordinates", img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


class CameraDTO:
    TOPIC_TEMPLATE = "/camera{}/color/camera_info"

    def __init__(self, index, pose=None, intrinsic_matrix=None, image=None):
        """
        Parameters
        ----------
        index: int
        pose: geometryMsgs/Pose
        intrinsic_matrix: np.ndarray
                [fx  0 cx]
            K = [ 0 fy cy]
                [ 0  0  1]
        image: np.ndarray

        """
        self.index = index
        self.pose = pose
        self.intrinsic_matrix = intrinsic_matrix
        self.image = image

        self.initialize()

    def initialize(self):
        if self.pose is None:
            self.pose = gconst.CAMERAS["camera{}".format(self.index)]
        if self.intrinsic_matrix is None:
            self.get_intrinsic_matrix()
        if self.image is None:
            self.get_raw_image()

    def get_intrinsic_matrix(self):
        camera_info = rospy.wait_for_message(CameraDTO.TOPIC_TEMPLATE.format(self.index), CameraInfo)
        self.intrinsic_matrix = np.reshape(camera_info.K, (3, 3))

    def get_raw_image(self):
        # See GenerateSchematic.get_image
        self.image = cv2.imread("images/camera{}_image_raw.jpg".format(self.index))


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
        sums[sums == 0] = 0.01
        return (img/sums * 255).astype(np.uint8)


    def match_images(self, image_indices):
        """
        Parameters
        ----------
        image_indices: list
            A list of the indices corresponding to the images we want to match.

        TODO:
        1. DONE Get intrinsic camera matrices from urdf file
        2. DONE Get R and T transform between cameras
        3. Fill in find_corners_3d to find corners in each image
        4. Given corners in each image, find corners that match in both images
        5. Use least squares triangulate to find 3d coordinates of these corners
        6. Fill in missing corners layer by layer (also, if corner shows up in layer n but not layer n - 1, add to layer n - 1.)
        7. Reconstruct positions of blocks from 3d coordinates of corners

        """
        cameras = [CameraDTO(index) for index in image_indices]

        #calculate R/T transform between cameras
        firstPose = cameras[0].pose
        secondPose = cameras[1].pose
        g1 = tr.quaternion_matrix([firstPose.orientation.x, firstPose.orientation.y, firstPose.orientation.z, firstPose.orientation.w])
        g1[0:3, -1] = [firstPose.position.x, firstPose.position.y, firstPose.position.z]
        g2 = tr.quaternion_matrix([secondPose.orientation.x, secondPose.orientation.y, secondPose.orientation.z, secondPose.orientation.w])
        g2[0:3, -1] = [secondPose.position.x, secondPose.position.y, secondPose.position.z]
        g = np.matmul(g1, np.linalg.inv(g2))
        R = g[0:3, 0:3]
        T = g[0:3, -1]
        print("R", R)
        print("T", T)

        #find matching corners in both images
        orb = cv2.ORB_create()
        first_keypoints, first_descriptors = orb.detectAndCompute(cameras[0].image, None)
        second_keypoints, second_descriptors = orb.detectAndCompute(cameras[1].image, None)
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(first_descriptors, second_descriptors)

        inlier_mask = np.array(matching.FilterByEpipolarConstraint(cameras[0].intrinsic_matrix, cameras[1].intrinsic_matrix, first_keypoints, second_keypoints, R, T, .11, matches)) == 1
        filtered_matches = [m for m,b in zip(matches, inlier_mask) if b == 1]

        print(len(filtered_matches))
        img3 = cv2.drawMatches(cameras[0].image,first_keypoints,cameras[1].image,second_keypoints,filtered_matches,None, flags=2)
        plt.imshow(img3)
        plt.show()


    def find_corners_3d(self, img):
        # edges = Segmentation.edge_detect_canny(img)
        #convert to grayscale and threshold so blocks appear white
        unified = self.unify_colors(img)
        segmented, clustered_segments, labels_bincount = self.segment(unified, segmentation_method=Segmentation.cluster_segment, n_clusters=11)
        total_labels = sum(labels_bincount)
        corners = []
        for i, segment in enumerate(clustered_segments):
            percent_data = labels_bincount[i]/float(total_labels)
            print(i)
            print(percent_data)
            #if this is a cluster we want to look at
            #(has percent_data within a certain range, indicating that the cluster has boxes)
            self.save_image(segment, "segmented_" + str(i) + ".jpg")
            if percent_data > .001 and percent_data < .5:
                gray = cv2.cvtColor(segment, cv2.COLOR_BGR2GRAY)
                features = cv2.goodFeaturesToTrack(gray, 4, .01, 10)
                for feature in features:
                    corners.append((feature[0][0], feature[0][1]))

        # for corner in corners:
        #      cv2.circle(img, corner, 3, (255, 255, 255), -1)
        # cv2.imshow("CornerCoordinates", img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        return corners



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
