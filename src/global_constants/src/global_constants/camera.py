import os

import rospy
import numpy as np

import tf.transformations as tr

from sensor_msgs.msg import CameraInfo

from global_constants import constants as gconst, utils as gutils


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
            self.pose = gconst.CAMERAS[CAMERA_NAME_TEMPLATE.format(self.index)]
        if self.intrinsic_matrix is None:
            self.get_intrinsic_matrix()
        if self.image is None:
            self.get_raw_image()

    def get_intrinsic_matrix(self):
        camera_info = rospy.wait_for_message(CameraDTO.TOPIC_TEMPLATE.format(self.index), CameraInfo)
        self.intrinsic_matrix = np.reshape(camera_info.K, (3, 3))

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
        orient, pos = pose.orientation, pose.position

        g = tr.quaternion_matrix([orient.x, orient.y, orient.z, orient.w])
        g[0:3, -1] = [pos.x, pos.y, pos.z]

        return g
