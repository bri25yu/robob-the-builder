#!/usr/bin/env python
import sys

import numpy as np

import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import os

from global_constants import constants as gconst, utils as gutils, camera


bridge = CvBridge()


def main():
    if not os.path.exists(gconst.IMG_DIR):
        os.makedirs(gconst.IMG_DIR)

    for i in range(len(camera.CAMERAS)):
        saveImage(camera.CameraDTO.IMAGE_TOPIC_TEMPLATE.format(i), camera.CameraDTO.IMAGE_SAVE_TEMPLATE.format(i))


def saveImage(topic, filename):
    imageData = rospy.wait_for_message(topic, Image)
    oldData = imageData
    while imageData == oldData:
        print("data is the same as old data!")
        imageData = rospy.wait_for_message(topic, Image)
        rospy.sleep(0.01)

    image = bridge.imgmsg_to_cv2(imageData, desired_encoding='passthrough')
    cv.imwrite(filename, image)
    print("Saved image from {} to {}".format(topic, filename))


if __name__ == "__main__":
    rospy.init_node('take_photo_node', anonymous=True)
    main()
