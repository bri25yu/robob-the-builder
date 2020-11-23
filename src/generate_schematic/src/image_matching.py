#!/usr/bin/env python

"""Code for Lab 7
Course: EECS C106A, Fall 2020
Author: Jay Monga
This file takes in a stream of data from a drone with known pose and generates a pointcloud.
"""

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped
import cv2
import ros_numpy
from ros_numpy import numpy_msg
import numpy as np
import tf.transformations as transformations
import tf2_ros as tf2

#!/usr/bin/env python
"""Code for Lab 7
Course: EECS C106A, Fall 2020
Author: Ritika Shrivatava
This file takes two images and their intrictics matrices as input and return matched features
between the two images with epipolar constraints.
"""
import cv2
import numpy as np

def epipolar_distance(x, l):
    return np.abs(x.dot(l)) / np.sqrt(l[0] ** 2 + l[1] ** 2)

def epipolar_error(x1, x2, l1, l2):
    """
       Task 2
       ------
       Computes the error of the candidate match (x1, x2), given in *normalized* image
       homogeneous coordinates. l1 and l2 are the respective epipolar lines.

       x1: np.array of size (3,): (u1, v1, 1)
       x2: np.array of size (3,): (u2, v2, 1)
       l1: np.array of size (3,): (a1, b1, c1)
       l2: np.array of size (3,): (a2, b2, c2)
    """
    # calculate the distance between the line l1 and x1.
    d1 = epipolar_distance(x1, l1)

    # calculate the distance between the line l2 and x2.
    d2 = epipolar_distance(x2, l2)

    # compute the total error.
    error = d1 + d2

    return error


def FilterByEpipolarConstraint(intrinsics1, intrinsics2, points1, points2, R, T, threshold, matches):
    """
        Task 2
        ------
        Returns an array inlier_mask of length equal to the length of matches. inlier_mask[i] is 1
        if matches[i] satisfies the epipolar constraint (i.e. the error is less than the threshold.
        Otherwise, inlier_mask[i] = 0.

        intrinsics1: np.array of size (3, 3): intrinsic camera matrix of left camera.
        intrinsics2: np.array of size (3, 3): intrinsic camera matrix of right camera.
        points1: np.array of size (M, 3): homogeneous, unnormalized coordinates of keypoints in left image.
        points2: np.array of size (N, 3): homogeneous, unnormalized coordinates of keypoints in right image.
        matches: list of cv2.Match objects. matches[i].queryIdx is the index in points1 of the first keypoint
                 in the i-th match. matches[i].trainIdx is the index in points2 of the second keypoint in the
                 i-th match.
    """
    # Delete this return statement when you implement this function.
    # return np.ones(len(matches)).astype(np.int32)

    # Compute Essential matrix
    T_hat = np.reshape(np.array([0, -T[2], T[1], T[2], 0, -T[0], -T[1], T[0], 0]), (3, 3))
    E = T_hat.T.dot(R)
    E = np.cross(T,R, axisa=0, axisb=0)

    inlier_mask = []

    for i in matches:
        u_v1 = points1[i.queryIdx]
        u_v2 = points2[i.trainIdx]

        (u1,v1) = u_v1.pt
        (u2,v2) = u_v2.pt

        # normalize x1 and x2
        p1 = np.array([u1, v1, 1])
        p2 = np.array([u2, v2, 1])
        x1 = np.linalg.inv(intrinsics1).dot(p1)
        x2 = np.linalg.inv(intrinsics2).dot(p2)

	    # compute epilines l1, l2.
        l1 = E.T.dot(x2)
        l2 = E.dot(x1)

        error = epipolar_error(x1, x2, l1, l2)
        m = (error < threshold).astype(int)
        inlier_mask.append(m)
    return np.array(inlier_mask)



def do_least_squares(A, T):
    return np.linalg.inv(A.T.dot(A)).dot(A.T).dot(T)

def least_squares_triangulate(x_1, x_2, R, T, left_intrinsic, right_intrinsic):
    """
    Task 3
    ------
    Computes the coordinates of the point represented by the corresponding pair (x1, x2).
    x1, x2 are given in unnormalized homogeneous coordinates.
    You should compute the coordinates X of the point written in the reference frame of the
    right camera.

    (R, T) is the transform g_21.

    left_intrinsic and right_intrinsic are both numpy arrays of size (3, 3) representing the
    3x3 intrinsic matrices of the left and right cameras respectively.
    """

    # Remove this return statement once you implement this function.
    # return None

    left_intrinsic_inv = np.linalg.inv(left_intrinsic)
    right_intrinsic_inv = np.linalg.inv(right_intrinsic)

    A = np.hstack((-R.dot(x_1), x_2))

    # Use least squares to solve for lambda1 and lambda2.
    lambda_1, lambda_2 = do_least_squares(A, T)

    if lambda_1 > 0 and lambda_2 > 0:
        X1 = lambda_2 * x_2
        X2 = lambda_1  * R.dot(x_1) + T
        X = .5 * (X1 + X2)
        return X
    else:
        return None
