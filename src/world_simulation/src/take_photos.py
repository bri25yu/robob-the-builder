#!/usr/bin/env python
import sys

import numpy as np

import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import os

from global_constants import constants as gconst, utils as gutils
from global_constants.camera import CameraDTO as camera


bridge = CvBridge()


def main():
    if not os.path.exists(gconst.IMG_DIR):
        os.makedirs(gconst.IMG_DIR)

    for i in range(len(gconst.CAMERAS)):
        saveImage(camera.IMAGE_TOPIC_TEMPLATE.format(i), camera.IMAGE_SAVE_TEMPLATE.format(i))


def saveImage(topic, filename):
    imageData = rospy.wait_for_message(topic, Image)
    image = bridge.imgmsg_to_cv2(imageData, desired_encoding='passthrough')
    gutils.save_image(image, filename)
    print("Saved image from {} to {}".format(topic, filename))


if __name__ == "__main__":
    rospy.init_node('take_photo_node', anonymous=True)
    main()
