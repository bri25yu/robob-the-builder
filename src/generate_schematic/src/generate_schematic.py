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
    camera3_coordinates = FeatureDetect.find_all_corners_3d(camera2, camera3).T
    # print(world_coordinates.shape)
    g03 = CameraDTO.get_g(camera3.pose)
    camera3_coordinates = camera3_coordinates.T
    camera3_coordinates = np.hstack((camera3_coordinates, np.ones((camera3_coordinates.shape[0], 1))))
    world_coordinates = g03.dot(camera3_coordinates.T).T
    # world_coordinates = world_coordinates[:, :3]

    ImageMatching.scatter3d(world_coordinates)

    img_coords = ImageMatching.project_3d_to_cam(world_coordinates[:, :3], camera3)
    ImageMatching.draw_points(camera3.image, img_coords, save_name="projected_points_on_image.jpg")

    with open(OUTPUT_FILE, "w") as file:
        for coord in world_coordinates:
            file.write("{}".format(coord))


if __name__ == "__main__":
    main()
