#!/usr/bin/env python
import sys

import numpy as np

import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import os

directory_to_save = 'images'
def saveImage(topic, filename):
    imageData = rospy.wait_for_message(topic, Image)
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(imageData, desired_encoding='passthrough')
    save_location = os.path.join(directory_to_save, filename)
    cv.imwrite(save_location, image)
    print("saved image to " + os.path.join(os.getcwd(), save_location))

if __name__ == "__main__":
    rospy.init_node('take_photo_node', anonymous = True)
    saveImage("/camera/color/image_raw", "gazebo_image_one.jpg")
    saveImage("/camera2/color/image_raw", "gazebo_image_two.jpg")
