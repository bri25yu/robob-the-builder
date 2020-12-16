from sensor_msgs.msg import Image
import cv2
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from global_constants.camera import CameraDTO
from global_constants import utils as gutils


bf = cv2.BFMatcher(cv2.NORM_HAMMING)


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
        T21 = np.reshape(g21[0:3, -1], (3, 1))

        # Find matching corners in both images
        matches = bf.match(des1, des2)
        inlier_mask = ImageMatching.FilterByEpipolarConstraint(camera1.intrinsic_matrix, camera2.intrinsic_matrix, kp1, kp2, R21, T21, epipolar_threshold, matches)
        filtered_matches = [m for m,b in zip(matches, inlier_mask) if b == 1]
        left_matches = np.array([kp1[filtered_matches[i].queryIdx].pt for i in range(len(filtered_matches))])
        right_matches = np.array([kp2[filtered_matches[i].trainIdx].pt for i in range(len(filtered_matches))])
        # ImageMatching.draw_matches(camera1.image, kp1, camera2.image, kp2, filtered_matches)
        coordinates = ImageMatching.get_matched_3d_coordinates(left_matches, right_matches, R21, T21, camera1.intrinsic_matrix, camera2.intrinsic_matrix)
        coordinates = np.reshape(coordinates, (len(coordinates), 3))
        return coordinates

    @staticmethod
    def scatter3d(points, ax=None, title=None):
        """
        Displays the given points on 3D axes.
        """
        if ax == None:
            fig = plt.figure()
            ax = Axes3D(fig)

        x_coords = points[:, 0]
        y_coords = points[:, 1]
        z_coords = points[:, 2]

        ax.scatter(x_coords, y_coords, z_coords)
        if title is not None:
            ax.set_title(title)

        if ax == None:
            plt.show()

    @staticmethod
    def draw_matches(image1, keypoints1, image2, keypoints2, matches):
        """
        Draws the given keypoints onto their respective images and displays them side by side.
        """
        img3 = cv2.drawMatches(image1, keypoints1, image2, keypoints2, matches, None, flags=2)
        plt.imshow(img3)
        plt.show()

    @staticmethod
    def draw_points(image, points, color=(255, 0, 0), save_name=None, display=True):
        """
        Draws the input points on the input image.

        Parameters
        ----------
        image: A (w, h, 3)-shaped np.ndarray
        points: list
            A list of points to draw on image.

        """
        image = image.copy()
        for px, py in points:
            u, v = int(px), int(py)
            image = cv2.circle(image, (u, v), radius=3, color=color, thickness=-1)
        plt.imshow(image)
        if save_name is not None:
            gutils.save_image(image, save_name)
        if display:
            plt.show()

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
        return ImageMatching.epipolar_distance(x1, l1) + ImageMatching.epipolar_distance(x2, l2)

    @staticmethod
    def lift(arr):
        """
        Parameters
        ----------
        arr: (n, d)-shaped np.ndarray

        Returns
        -------
        arr_lifted: (n, d+1)-shaped np.ndarray
            arr with ones appended for each coordinate.

        """
        return np.hstack((arr, np.ones((len(arr), 1))))

    @staticmethod
    def apply_transform(coordinates, transform):
        """
        Parameters
        ----------
        coordinates: (n, d)-shaped np.ndarray
        transform: (d, d)-shaped np.ndarray

        """
        return transform.dot(coordinates.T).T

    @staticmethod
    def get_matched_3d_coordinates(left_matches, right_matches, R, T, left_intrinsic, right_intrinsic):
        """
        Computes the coordinates of the point represented by each pair of matches.
        x1, x2 are given in unnormalized homogeneous coordinates.
        You should compute the coordinates X of the point written in the reference frame of the
        right camera.

        (R, T) is the transform g_21.

        Parameters
        ----------
        left_matches: (n, 2)-shaped np.ndarray
        right_matches: (n, 2)-shaped np.ndarray
        R: (3, 3)-shaped np.ndarray
        T: (3, 1)-shaped np.ndarray
        left_intrinsic: (3, 3)-shaped np.ndarray
        right_intrinsic: (3, 3)-shaped np.ndarray

        """
        left_matches, right_matches = ImageMatching.lift(left_matches), ImageMatching.lift(right_matches)

        left_matches = ImageMatching.apply_transform(left_matches, np.linalg.inv(left_intrinsic))
        right_matches = ImageMatching.apply_transform(right_matches, np.linalg.inv(right_intrinsic))

        left_matches_rotated = ImageMatching.apply_transform(left_matches, R)

        A = np.empty((2 * len(left_matches), 3))
        A[0::2], A[1::2] = -left_matches_rotated, right_matches
        A = A.T

        lambdas = np.hstack(np.linalg.lstsq(A[:, 2*i: 2*i+2], T)[0] for i in range(len(left_matches)))
        indices = np.ravel(np.argwhere((lambdas[0] > 0) & (lambdas[1] > 0)))
        left_matches_rotated, right_matches = left_matches_rotated[indices], right_matches[indices]
        lambdas = lambdas[:, indices].T

        X1 = lambdas[:, 0:1] * right_matches
        X2 = (lambdas[:, 1:2] * left_matches_rotated) + np.ravel(T)
        return .5 * (X1 + X2)
