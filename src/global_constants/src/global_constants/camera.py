import os

import rospy
import numpy as np

import tf.transformations as tr

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

from global_constants import constants as gconst, utils as gutils


CAMERAS = dict()
for i, d in enumerate(gconst.CAMERA_DATA):
    CAMERAS["camera{}".format(i)] = Pose(position=Point(*d[0]), orientation=Quaternion(*gutils.rpy_to_quaternion(*d[1])))


class CameraDTO:
    TOPIC_TEMPLATE = "/camera{}/color/camera_info"
    IMAGE_TOPIC_TEMPLATE = "/camera{}/color/image_raw"
    IMAGE_SAVE_TEMPLATE = os.path.join(gconst.IMG_DIR, "camera{}_image_raw.jpg")
    CAMERA_NAME_TEMPLATE = "camera{}"

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
            self.pose = CAMERAS[CameraDTO.CAMERA_NAME_TEMPLATE.format(self.index)]
        if self.intrinsic_matrix is None:
            self.get_intrinsic_matrix()
        if self.image is None:
            self.get_raw_image()

    def get_intrinsic_matrix(self):
        self.intrinsic_matrix = gutils.input_object(gconst.KINECT_INTRINSIC_MATRIX_FILE)

    def get_raw_image(self):
        self.image = gutils.get_image(CameraDTO.IMAGE_SAVE_TEMPLATE.format(self.index))

    @staticmethod
    def get_transformation(camera1, camera2):
        """
        Parameters
        ----------
        camera1: CameraDTO
        camera2: CameraDTO

        Returns
        -------
        g21: np.ndarray
            A (4, 4)-shaped array representing the transformation from camera1 coordinates to camera2 coordinates.
        g01: np.ndarray
            A (4, 4)-shaped array representing the transformation from camera1 coordinates to world frame coordinates.
        g02: np.ndarray
            A (4, 4)-shaped array representing the transformation from camera2 coordinates to world frame coordinates.

        """
        g01 = CameraDTO.get_g(camera1.pose)
        g02 = CameraDTO.get_g(camera2.pose)

        g21 = np.matmul(np.linalg.inv(g02), g01)

        return g21, g01, g02

    # Helpers--------------------------------------------------------------------------------------

    @staticmethod
    def get_g(pose):
        """
        Parameters
        ----------
        pose: Pose

        Returns
        -------
        g: np.ndarray
            A (4, 4)-shaped array representing the transformation from this pose to world frame coordinates.

        """
        #note: i think this is wrong
        orient, pos = pose.orientation, pose.position

        g = tr.quaternion_matrix([orient.x, orient.y, orient.z, orient.w])
        g[0:3, -1] = [pos.x, pos.y, pos.z]
        transformation = np.array([[0, -1, 0, 0],
                                        [0, 0, -1, 0],
                                        [1, 0, 0, 0],
                                        [0, 0, 0, 1]])
        return g.dot(np.linalg.inv(transformation))
