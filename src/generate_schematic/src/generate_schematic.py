#!/usr/bin/env python

import rospy
import cv2
import numpy as np

from feature_detect import FeatureDetect
from global_constants.camera import CameraDTO
from image_matching import ImageMatching
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


IMAGE_IN_PATH = "images/gazebo_angled_image_one.jpg"
IMAGE_OUT_NAME = "gazebo_angled_image_one_clustering.jpg"

OUTPUT_FILE = "output/schematic.txt"

def main():
    rospy.init_node("schematic_node", anonymous = True)
    #idea: take all points and round coordinates to nearest multiples
    world_coordinates = []
    for i in range(12):
        new_pair_coordinates = get_coordinates_for_pair(2 * i, 2 * i + 1)
        if len(new_pair_coordinates) != 0:
            world_coordinates.append(new_pair_coordinates)
    world_coordinates = np.vstack(world_coordinates)

    #remove duplicates
    world_coordinates = unique_rows(world_coordinates)
    before_apply_square_world_coordinates = world_coordinates.copy()[world_coordinates[:, 2] >= 0]

    #go downward layer by layer, and at each layer remove non-squares and project points to lower layers
    max_z_coordinate = max(world_coordinates[:, 2])
    z = max_z_coordinate
    z_diff = .12
    filtered_layers = []
    while (z >= 0):
        cur_layer = np.array([c for c in world_coordinates if c[2] == z])
        cur_layer = apply_square_filter(cur_layer)
        if len(cur_layer) != 0:
            filtered_layers.append(cur_layer)
        if z != 0:
            for point in cur_layer:
                world_coordinates = np.vstack((world_coordinates, np.array([point[0], point[1], z - z_diff])))
        world_coordinates = unique_rows(world_coordinates)
        z -= z_diff

    world_coordinates = np.vstack(filtered_layers)

    #add points from first layer to second layer
    first_layer = np.array([c for c in world_coordinates if c[2] == 0])
    new_second_layer_points = np.array([[c[0], c[1], c[2] + z_diff] for c in first_layer])
    world_coordinates = np.vstack((world_coordinates, new_second_layer_points))
    world_coordinates = unique_rows(world_coordinates)

    fig = plt.figure(figsize=plt.figaspect(0.5))
    ax1 = fig.add_subplot(1, 3, 1, projection='3d')
    ax2 = fig.add_subplot(1, 3, 2, projection='3d')
    ImageMatching.scatter3d(before_apply_square_world_coordinates, ax1)
    ImageMatching.scatter3d(world_coordinates, ax2)
    plt.show()


def apply_square_filter(coordinates):
    coordinate_set = set()
    for coordinate in coordinates:
        coordinate_set.add(tuple(coordinate))
    x_diff, y_diff = 0.06, 0.06
    #remove points that aren't part of squares
    filtered_coordinates = []
    for coordinate in coordinates:
        x, y, z = coordinate[0], coordinate[1], coordinate[2]
        possible_squares = [[(x + x_diff, y + y_diff, z), (x + x_diff, y, z), (x, y + y_diff, z)],
                            [(x - x_diff, y, z), (x - x_diff, y - y_diff, z), (x, y - y_diff, z)],
                            [(x + x_diff, y, z), (x, y - y_diff, z), (x + x_diff, y - y_diff, z)],
                            [(x - x_diff, y, z), (x, y + y_diff, z), (x - x_diff, y + y_diff, z)]]
        for square in possible_squares:
            square_corners = [square[i] in coordinate_set for i in range(3)]
            if sum(square_corners) >= 3:
                filtered_coordinates.append(coordinate)
    return np.array(filtered_coordinates)

def unique_rows(a):
    a = np.ascontiguousarray(a)
    unique_a = np.unique(a.view([('', a.dtype)]*a.shape[1]))
    return unique_a.view(a.dtype).reshape((unique_a.shape[0], a.shape[1]))

def get_coordinates_for_pair(n1, n2):
    camera2, camera3 = CameraDTO(n1), CameraDTO(n2)
    camera3_coordinates = FeatureDetect.find_all_corners_3d(camera2, camera3, epipolar_threshold=0.01).T
    g03 = CameraDTO.get_g(camera3.pose)
    camera3_coordinates = camera3_coordinates.T
    world_coordinates = ImageMatching.apply_transform(ImageMatching.lift(camera3_coordinates), g03)[:, :3]
    #remove points with z-coordinate that doesn't make sense
    tolerances = np.array([0.01, 0.01, 0.02])
    offset = np.array([2, 0, 0])
    multiple = np.array([0.06, 0.06, 0.12])
    world_coordinates = world_coordinates[close_to_multiples_of(world_coordinates, multiple, offset, tolerances)]
    return round_nearest(world_coordinates, offset, multiple)

def round_nearest(number, offset, multiple):
    return np.round((number - offset) / multiple) * multiple + offset

def close_to_multiples_of(coordinates, multiple, offset, tolerance=.05):
    dists = np.abs(((coordinates - offset) % multiple) - (multiple / 2)) - (multiple / 2)
    close = np.ones(len(coordinates))
    for i in range(coordinates.shape[1]):
        close = close * np.isclose(dists[:, i], 0, atol=tolerance[i])
    return np.ravel(np.argwhere(close))

if __name__ == "__main__":
    main()
