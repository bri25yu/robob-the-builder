#!/usr/bin/env python

import rospy
import cv2

from feature_detect import FeatureDetect
from camera import CameraDTO


IMAGE_IN_PATH = "images/gazebo_angled_image_one.jpg"
IMAGE_OUT_NAME = "gazebo_angled_image_one_clustering.jpg"

OUTPUT_FILE = "output/schematic.txt"

def main():
    rospy.init_node("schematic_node", anonymous = True)

    camera2, camera3 = CameraDTO(2), CameraDTO(3)
    coordinates = FeatureDetect.find_all_corners_3d(camera2, camera3)

    with open(OUTPUT_FILE, "w") as file:
        for coord in coordinates:
            file.write("{}".format(coord))


if __name__ == "__main__":
    main()
