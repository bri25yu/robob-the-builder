#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv


bridge = CvBridge()


def saveImage(topic, filename):
    start_time = rospy.get_time()
    while True:
        imageData = rospy.wait_for_message(topic, Image)
        if imageData.header.stamp.secs > start_time:
            break
    image = bridge.imgmsg_to_cv2(imageData, desired_encoding='passthrough')
    cv.imwrite(filename, image)
    print("Saved image from {} to {}".format(topic, filename))
