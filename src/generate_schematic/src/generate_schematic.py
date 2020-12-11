#!/usr/bin/env python

import rospy
import cv2
import numpy as np

from feature_detect import FeatureDetect
from global_constants.camera import CameraDTO
from image_matching import ImageMatching
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from global_constants import utils as gutils, constants as gconst


IMAGE_IN_PATH = "images/gazebo_angled_image_one.jpg"
IMAGE_OUT_NAME = "gazebo_angled_image_one_clustering.jpg"

Z_DIFF_3D = np.array([0, 0, gconst.BLOCK_Z])

def main():
    rospy.init_node("schematic_node", anonymous = True)
    #idea: take all points and round coordinates to nearest multiples
    raw_world_coordinates, world_coordinates = get_world_coordinates()
    world_coordinates = np.array([c for c in world_coordinates if c[2] >= 0])
    print(world_coordinates)
    before_apply_square_world_coordinates = world_coordinates.copy()[world_coordinates[:, 2] >= 0]

    #go downward layer by layer, and at each layer remove non-squares and project points to lower layers
    max_z_coordinate = max(world_coordinates[:, 2])
    z = max_z_coordinate
    z_diff = gconst.BLOCK_Z
    layers = gutils.get_layers(world_coordinates)
    filtered_layers = []
    for i in reversed(range(len(layers))):
        print("i", i)
        cur_layer = apply_square_filter(layers[i])
        print(cur_layer)
        if len(cur_layer) != 0:
            filtered_layers.append(cur_layer)
        if not np.isclose(layers[i][0][2], 0) and len(cur_layer) != 0:
            layers[i-1] = np.vstack((layers[i-1], cur_layer - Z_DIFF_3D))
    world_coordinates = np.vstack(filtered_layers)
    world_coordinates = gutils.unique_rows(world_coordinates)

    #add points from first layer to second layer
    first_layer = np.array([c for c in world_coordinates if np.isclose(c[2], 0)])
    new_second_layer_points = first_layer + Z_DIFF_3D
    world_coordinates = np.vstack((world_coordinates, new_second_layer_points))
    world_coordinates = gutils.round_nearest(world_coordinates, gconst.offset, gconst.multiple)
    world_coordinates = gutils.unique_rows(world_coordinates)
    print("world_coordinates", world_coordinates)

    layers_to_output = gutils.get_layers(world_coordinates)[1:]
    bottom_left_corners = np.vstack(gutils.get_bottom_left_corners(layer) for layer in layers_to_output) - Z_DIFF_3D
    print(bottom_left_corners)
    gutils.output_corners(bottom_left_corners)
    print("bottom left corners read in", gutils.get_corners())

    fig = plt.figure(figsize=plt.figaspect(0.5))
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    ax3 = fig.add_subplot(2, 2, 3, projection='3d')
    ax4 = fig.add_subplot(2, 2, 4, projection='3d')
    ImageMatching.scatter3d(raw_world_coordinates, ax1, title="Raw world coordinates")
    ImageMatching.scatter3d(before_apply_square_world_coordinates, ax2, title="World coords before filtering")
    ImageMatching.scatter3d(world_coordinates, ax3, title="World coords after processing")
    ImageMatching.scatter3d(bottom_left_corners, ax4, title="Bottom left corners sent to robot")
    plt.savefig("output/world_coordinates.jpg")
    plt.show()


def get_world_coordinates():
    world_coordinates = []
    raw_world_coordinates = []
    for i in range(len(gconst.CAMERA_DATA) // 2):
        raw_pair_coordinates = get_coordinates_for_pair(2 * i, 2 * i + 1)

        potential_grid_indices = gutils.close_to_multiples_of(raw_pair_coordinates, gconst.multiple, gconst.offset, gconst.tolerances)
        new_pair_coordinates = raw_pair_coordinates[potential_grid_indices]

        new_pair_coordinates = gutils.round_nearest(new_pair_coordinates, gconst.offset, gconst.multiple)

        if len(new_pair_coordinates) != 0:
            world_coordinates.append(new_pair_coordinates)
            raw_world_coordinates.append(raw_pair_coordinates)
    world_coordinates = np.vstack(world_coordinates)
    raw_world_coordinates = np.vstack(raw_world_coordinates)

    #remove duplicates
    world_coordinates = gutils.unique_rows(world_coordinates)

    return raw_world_coordinates, world_coordinates

def get_coordinates_for_pair(n1, n2):
    camera2, camera3 = CameraDTO(n1), CameraDTO(n2)
    camera3_coordinates = FeatureDetect.find_all_corners_3d(camera2, camera3, epipolar_threshold=0.01).T
    g03 = CameraDTO.get_g(camera3.pose)
    camera3_coordinates = camera3_coordinates.T
    world_coordinates = ImageMatching.apply_transform(ImageMatching.lift(camera3_coordinates), g03)[:, :3]
    #remove points with z-coordinate that doesn't make sense
    return world_coordinates


def apply_square_filter(coordinates):
    coordinate_set = set()
    for coordinate in coordinates:
        coordinate_set.add(tuple(coordinate))
    x_diff, y_diff = gconst.BLOCK_X, gconst.BLOCK_Y
    #remove points that aren't part of squares
    filtered_coordinates = []
    for coordinate in coordinates:
        x, y, z = coordinate[0], coordinate[1], coordinate[2]
        possible_squares = [[(x + x_diff, y + y_diff, z), (x + x_diff, y, z), (x, y + y_diff, z)],
                            [(x - x_diff, y, z), (x - x_diff, y - y_diff, z), (x, y - y_diff, z)],
                            [(x + x_diff, y, z), (x, y - y_diff, z), (x + x_diff, y - y_diff, z)],
                            [(x - x_diff, y, z), (x, y + y_diff, z), (x - x_diff, y + y_diff, z)]]
        for square in possible_squares:
            square_corners = 0
            for i in range(3):
                for c in coordinates:
                    if np.allclose(c, square[i]):
                        square_corners += 1
                        break
            if square_corners >= 3:
                filtered_coordinates.append(coordinate)
    return np.array(filtered_coordinates)


if __name__ == "__main__":
    main()
