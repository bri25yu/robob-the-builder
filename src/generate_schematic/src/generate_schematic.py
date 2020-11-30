#!/usr/bin/env python

import rospy
import cv2


IMAGE_IN_PATH = "images/gazebo_angled_image_one.jpg"
IMAGE_OUT_NAME = "gazebo_angled_image_one_clustering.jpg"

def main():
    rospy.init_node("schematic_node", anonymous = True)
    g = GenerateSchematic()
    coordinates = g.match_images([2, 3])
    # img = g.get_image(IMAGE_IN_PATH)
    # bottom_left_coordinates = g.find_all_bottom_left_coordinates_2d(img)
    #
    # #display an image with bottom left coordinates highlighted in white
    # for coordinates in bottom_left_coordinates:
    #     cv2.circle(img, coordinates, 3, (255, 255, 255), -1)
    # cv2.imshow("BottomLeftCoordinates", img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
