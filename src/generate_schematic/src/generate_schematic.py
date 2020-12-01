#!/usr/bin/env python

import rospy
import cv2
import numpy as np

from feature_detect import FeatureDetect
from global_constants.camera import CameraDTO
from image_matching import ImageMatching


IMAGE_IN_PATH = "images/gazebo_angled_image_one.jpg"
IMAGE_OUT_NAME = "gazebo_angled_image_one_clustering.jpg"

OUTPUT_FILE = "output/schematic.txt"

def main():
    rospy.init_node("schematic_node", anonymous = True)

    camera2, camera3 = CameraDTO(2), CameraDTO(3)
    coordinates = FeatureDetect.find_all_corners_3d(camera2, camera3)

    g30 = CameraDTO.get_g(camera3.pose)
    world_coordinates = np.hstack((coordinates, np.ones((len(coordinates), 1))))
    world_coordinates = np.linalg.inv(g30).dot(world_coordinates).T
    world_coordinates = world_coordinates[:, :3]

    ImageMatching.scatter3d(world_coordinates)

    img_coords = ImageMatching.project_3d_to_cam(coordinates.T, camera3)
    ImageMatching.draw_points(camera3.image, img_coords, save_name="projected_points_on_image.jpg")

    with open(OUTPUT_FILE, "w") as file:
        for coord in world_coordinates:
            file.write("{}".format(coord))


if __name__ == "__main__":
    main()
