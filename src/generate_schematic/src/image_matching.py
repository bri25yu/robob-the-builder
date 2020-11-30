from sensor_msgs.msg import Image
import cv2
import numpy as np


class ImageMatching:

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
            m = (error < threshold).astype(int)
            inlier_mask.append(m)
        return np.array(inlier_mask)

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
        left_intrinsic_inv = np.linalg.inv(left_intrinsic)
        right_intrinsic_inv = np.linalg.inv(right_intrinsic)
        x_1 = np.reshape(x_1, (3, 1))
        x_2 = np.reshape(x_2, (3, 1))
        x_1 = np.matmul(left_intrinsic_inv, x_1)
        x_2 = np.matmul(right_intrinsic_inv, x_2)

        A = np.concatenate((-np.matmul(R, x_1), x_2), axis = 1)
        # Use least squares to solve for lambda1 and lambda2.
        lambda_1, lambda_2 = np.linalg.lstsq(A, T)[0]
        x_1 = np.reshape(x_1, (3,))
        x_2 = np.reshape(x_2, (3,))

        if lambda_1 > 0 and lambda_2 > 0:
            X1 = lambda_2 * x_2
            X2 = (lambda_1  * np.matmul(R, x_1)) + T
            X = .5 * (X1 + X2)
            return X
        else:
            return None

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

    # Helpers--------------------------------------------------------------------------------------
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
