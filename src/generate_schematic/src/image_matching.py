from sensor_msgs.msg import Image
import cv2
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from global_constants import utils as gutils
from global_constants.camera import CameraDTO


bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)


class ImageMatching:

    @staticmethod
    def match_2_cameras_images(camera1, camera2, kp1, des1, kp2, des2, epipolar_threshold=0.11):
        """
        Parameters
        ----------
        camera1: CameraDTO
        camera2: CameraDTO

        Returns
        -------
        coordinates: list
            A list of coordinates filtered by epipolar constraints.

        """
        # Calculate R/T transform between cameras
        g21, g01, g02 = CameraDTO.get_transformation(camera1, camera2)
        R21 = g21[0:3, 0:3]
        T21 = g21[0:3, -1]

        # Find matching corners in both images
        matches = bf.match(des1, des2)
        inlier_mask = ImageMatching.FilterByEpipolarConstraint(camera1.intrinsic_matrix, camera2.intrinsic_matrix, kp1, kp2, R21, T21, .01, matches)
        filtered_matches = [m for m,b in zip(matches, inlier_mask) if b == 1]
        left_matches = [kp1[filtered_matches[i].queryIdx].pt for i in range(len(filtered_matches))]
        right_matches = [kp2[filtered_matches[i].trainIdx].pt for i in range(len(filtered_matches))]
        coordinates = ImageMatching.get_matched_3d_coordinates(left_matches, right_matches, R21, T21, camera1.intrinsic_matrix, camera2.intrinsic_matrix)
        coordinates = np.reshape(coordinates, (len(coordinates), 3))
        # left_projection = np.matmul(camera1.intrinsic_matrix, np.linalg.inv(camera1.get_g(camera1.pose))[:3])
        # right_projection =  np.matmul(camera2.intrinsic_matrix, np.linalg.inv(camera2.get_g(camera2.pose))[:3])
        return coordinates

    @staticmethod
    def draw_matches(image1, keypoints1, image2, keypoints2, matches):
        img3 = cv2.drawMatches(image1, keypoints1, image2, keypoints2, matches, None, flags=2)
        plt.imshow(img3)
        plt.show()

    @staticmethod
    def draw_points(image, points, color=(255, 0, 0), save_name=None):
        image = image.copy()
        for px, py in points:
            u, v = int(px), int(py)
            image = cv2.circle(image, (u, v), radius=3, color=color, thickness=-1)
        plt.imshow(image)
        if save_name is not None:
            gutils.save_image(image, save_name)
        plt.show()

    @staticmethod
    def scatter3d(points, ax=None):
        if ax == None:
            fig = plt.figure()
            ax = Axes3D(fig)

        x_coords = points[:, 0]
        y_coords = points[:, 1]
        z_coords = points[:, 2]

        ax.scatter(x_coords, y_coords, z_coords)
        if ax == None:
            plt.show()

    @staticmethod
    def project_3d_to_cam(coords, camera):
        """
        Parameters
        ----------
        coords: list
            A list of (3, 1) coordinates in the world frame.
        camera: CameraDTO

        Returns
        -------
        cam_coords: list
            The input coords in the camera world.

        """
        intrinsic = camera.intrinsic_matrix
        g = CameraDTO.get_g(camera.pose)
        cam_coords = []
        for coord in coords:
            # coord = [2.06, -.06, 0]
            new_coord = np.linalg.inv(g).dot(np.hstack((coord, [1])))[:3]
            cam_coord = intrinsic.dot(new_coord)
            result = cam_coord[:2] / new_coord[2]
            cam_coords.append(result)
        return cam_coords

    # Helpers--------------------------------------------------------------------------------------

    @staticmethod
    def FilterByEpipolarConstraint(intrinsics1, intrinsics2, points1, points2, R, T, threshold, matches):
        """
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

            error = ImageMatching.epipolar_error(x1, x2, l1, l2)
            # print(error)
            m = (error < threshold).astype(int)
            inlier_mask.append(m)
        return np.array(inlier_mask)

    @staticmethod
    def epipolar_distance(x, l):
        return np.abs(x.dot(l)) / np.sqrt(l[0] ** 2 + l[1] ** 2)

    @staticmethod
    def epipolar_error(x1, x2, l1, l2):
        """
        Parameters
        ----------
        x1: np.ndarray
            A (3,)-shaped array representing the potentially matching point in image 1, given in
            normalized image homogeneous coordinates, (u1, v1, 1).
        x2: np.ndarray
            A (3,)-shaped array representing the potentially matching point in image 2, given in
            normalized image homogeneous coordinates, (u2, v2, 1).
        l1: np.ndarray
            A (3,)-shaped array representing the epipolar line for point 1, (a1, b1, c1).
        l2: np.ndarray
            A (3,)-shaped array representing the epipolar line for point 2, (a2, b2, c2).

        Returns
        -------
        error: float
            The error of the candidate match.

        """
        # calculate the distance between the line l1 and x1.
        d1 = ImageMatching.epipolar_distance(x1, l1)

        # calculate the distance between the line l2 and x2.
        d2 = ImageMatching.epipolar_distance(x2, l2)

        # compute the total error.
        error = d1 + d2

        return error

    @staticmethod
    def get_matched_3d_coordinates(left_matches, right_matches, R, T, left_intrinsic, right_intrinsic):
        coordinates = []
        for i in range(len(left_matches)):
            result = ImageMatching.least_squares_triangulate(left_matches[i] + (1,), right_matches[i] + (1,), R, T, left_intrinsic, right_intrinsic)
            if result is not None:
                coordinates.append(result)
        print("coordinate length", len(coordinates))
        return coordinates

    @staticmethod
    def least_squares_triangulate(x_1, x_2, R, T, left_intrinsic, right_intrinsic):
        """
        Computes the coordinates of the point represented by the corresponding pair (x1, x2).
        x1, x2 are given in unnormalized homogeneous coordinates.
        You should compute the coordinates X of the point written in the reference frame of the
        right camera.

        (R, T) is the transform g_21.

        left_intrinsic and right_intrinsic are both numpy arrays of size (3, 3) representing the
        3x3 intrinsic matrices of the left and right cameras respectively.
        """
        # print("R", R)
        # print("T", T)
        left_intrinsic_inv = np.linalg.inv(left_intrinsic)
        right_intrinsic_inv = np.linalg.inv(right_intrinsic)
        x_1 = np.reshape(x_1, (3, 1))
        x_2 = np.reshape(x_2, (3, 1))
        x_1 = np.matmul(left_intrinsic_inv, x_1)
        x_2 = np.matmul(right_intrinsic_inv, x_2)

        A = np.concatenate((-np.matmul(R, x_1), x_2), axis = 1)
        # Use least squares to solve for lambda1 and lambda2.
        lambda_1, lambda_2 = np.linalg.lstsq(A, T)[0]
        # print(lambda_1, lambda_2)
        x_1 = np.reshape(x_1, (3,))
        x_2 = np.reshape(x_2, (3,))
        if lambda_1 > 0 and lambda_2 > 0:
            X1 = lambda_2 * x_2
            X2 = (lambda_1  * np.matmul(R, x_1)) + T
            X = .5 * (X1 + X2)
            return X
        else:
            return None
