#!/usr/bin/env python

import rospy

from sensor_msgs.msg import CameraInfo

import numpy as np

import os

import cv2

import image_matching as matching

import matplotlib.pyplot as plt

from global_constants import constants as gconst, utils as gutils

import tf.transformations as tr


IMAGE_IN_PATH = "images/gazebo_angled_image_one.jpg"
IMAGE_OUT_NAME = "gazebo_angled_image_one_clustering.jpg"

def main():
    rospy.init_node("schematic_node", anonymous = True)
    g = GenerateSchematic()
    coordinates = g.match_images([2, 3])
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
        self.image = gutils.get_image("images/camera{}_image_raw.jpg".format(self.index))


class GenerateSchematic:

    def get_matched_3d_coordinates(self, left_matches, right_matches, R, T, left_intrinsic, right_intrinsic):
        coordinates = []
        for i in range(len(left_matches)):
            result = matching.least_squares_triangulate(left_matches[i] + (1,), right_matches[i] + (1,), R, T, left_intrinsic, right_intrinsic)
            if result is not None:
                coordinates.append(result)
        return coordinates

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
        left_matches = [first_keypoints[filtered_matches[i].queryIdx].pt for i in range(len(filtered_matches))]
        right_matches = [second_keypoints[filtered_matches[i].trainIdx].pt for i in range(len(filtered_matches))]

        coordinates = self.get_matched_3d_coordinates(left_matches, right_matches, R, T, cameras[0].intrinsic_matrix, cameras[1].intrinsic_matrix)

        coordinates = np.reshape(coordinates, (len(coordinates), 3))
        from mpl_toolkits.mplot3d import Axes3D
        import random

        fig = plt.figure()
        ax = Axes3D(fig)

        sequence_containing_x_vals = coordinates[:, 0]
        sequence_containing_y_vals = coordinates[:, 1]
        sequence_containing_z_vals = coordinates[:, 2]

        random.shuffle(sequence_containing_x_vals)
        random.shuffle(sequence_containing_y_vals)
        random.shuffle(sequence_containing_z_vals)

        ax.scatter(sequence_containing_x_vals, sequence_containing_y_vals, sequence_containing_z_vals)
        plt.show()


        return coordinates
        # img3 = cv2.drawMatches(cameras[0].image,first_keypoints,cameras[1].image,second_keypoints,filtered_matches,None, flags=2)
        # plt.imshow(img3)
        # plt.show()


if __name__ == "__main__":
    main()
