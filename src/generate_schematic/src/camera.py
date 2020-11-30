import rospy
import numpy as np

from sensor_msgs.msg import CameraInfo

from global_constants import constants as gconst, utils as gutils


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
