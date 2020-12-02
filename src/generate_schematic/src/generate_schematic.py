#!/usr/bin/env python

import rospy
import cv2
import numpy as np

from feature_detect import FeatureDetect
from global_constants.camera import CameraDTO
from image_matching import ImageMatching
import matplotlib.pyplot as plt

IMAGE_IN_PATH = "images/gazebo_angled_image_one.jpg"
IMAGE_OUT_NAME = "gazebo_angled_image_one_clustering.jpg"

OUTPUT_FILE = "output/schematic.txt"

def main():
    rospy.init_node("schematic_node", anonymous = True)

    camera2, camera3 = CameraDTO(4), CameraDTO(5)
    camera3_coordinates = FeatureDetect.find_all_corners_3d(camera2, camera3).T
    g03 = CameraDTO.get_g(camera3.pose)
    camera3_coordinates = camera3_coordinates.T
    camera3_coordinates = np.hstack((camera3_coordinates, np.ones((camera3_coordinates.shape[0], 1))))
    world_coordinates = g03.dot(camera3_coordinates.T).T
    world_coordinates = world_coordinates[np.ravel(close_to_multiples_of(world_coordinates[:, 2], .12))]
    print("world coordinates length", len(world_coordinates))
    ImageMatching.scatter3d(world_coordinates)
    img_coords = ImageMatching.project_3d_to_cam(world_coordinates[:, :3], camera2)
    ImageMatching.draw_points(camera2.image, img_coords, save_name="projected_points_on_image.jpg")

    with open(OUTPUT_FILE, "w") as file:
        for coord in world_coordinates:
            file.write("{}".format(coord))

def close_to_multiples_of(coordinates, n, tolerance = .05):
    return np.argwhere(np.isclose(coordinates % n, 0, rtol = tolerance) | np.isclose(coordinates % n, n, rtol = tolerance))

if __name__ == "__main__":
    main()
