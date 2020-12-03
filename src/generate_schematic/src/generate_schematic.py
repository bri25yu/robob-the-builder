#!/usr/bin/env python

import rospy
import cv2
import numpy as np

from feature_detect import FeatureDetect
from global_constants.camera import CameraDTO
from image_matching import ImageMatching
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.cluster import DBSCAN


IMAGE_IN_PATH = "images/gazebo_angled_image_one.jpg"
IMAGE_OUT_NAME = "gazebo_angled_image_one_clustering.jpg"

OUTPUT_FILE = "output/schematic.txt"

def main():
    rospy.init_node("schematic_node", anonymous = True)
    #idea: take all points and round coordinates to nearest multiples
    world_coordinates = get_coordinates_for_pair(0, 1)
    for i in range(1, 4):
        new_pair_coordinates = get_coordinates_for_pair(2 * i, 2 * i + 1)
        if len(new_pair_coordinates) != 0:
            world_coordinates = np.vstack((world_coordinates, new_pair_coordinates))

    #remove duplicates
    world_coordinates = unique_rows(world_coordinates)

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
        print(world_coordinates)

    world_coordinates = filtered_layers[0]
    for i in range(1, len(filtered_layers)):
        world_coordinates = np.vstack((world_coordinates, filtered_layers[i]))

    #add points from first layer to second layer
    first_layer = np.array([c for c in world_coordinates if c[2] == 0])
    new_second_layer_points = np.array([[c[0], c[1], c[2] + z_diff] for c in first_layer])
    world_coordinates = np.vstack((world_coordinates, new_second_layer_points))
    world_coordinates = unique_rows(world_coordinates)

    ImageMatching.scatter3d(np.array(world_coordinates))



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
    camera3_coordinates = FeatureDetect.find_all_corners_3d(camera2, camera3).T
    g03 = CameraDTO.get_g(camera3.pose)
    camera3_coordinates = camera3_coordinates.T
    camera3_coordinates = np.hstack((camera3_coordinates, np.ones((camera3_coordinates.shape[0], 1))))
    world_coordinates = g03.dot(camera3_coordinates.T).T
    #remove points with z-coordinate that doesn't make sense
    world_coordinates = world_coordinates[np.ravel(close_to_multiples_of(world_coordinates[:, 2], .12, 0, .02))]
    world_coordinates = world_coordinates[np.ravel(close_to_multiples_of(world_coordinates[:, 1], .06, 0, .02))]
    world_coordinates = world_coordinates[np.ravel(close_to_multiples_of(world_coordinates[:, 0], .06, 2, .02))]


    rounded_world_coordinates = []
    for coordinate in world_coordinates:
        x_rounded = round_nearest(coordinate[0], 2, 0.06)
        y_rounded = round_nearest(coordinate[1], 0, 0.06)
        z_rounded = round_nearest(coordinate[2], 0, 0.12)
        rounded_world_coordinates.append([x_rounded, y_rounded, z_rounded])


    return np.array(rounded_world_coordinates)

def reject_outliers(data, m=2):
    return data[abs(data - np.mean(data)) < m * np.std(data)]

def round_nearest(number, offset, multiple):
    return round((number - offset) / multiple) * multiple + offset

def close_to_multiples_of(coordinates, multiple, offset, tolerance = .05):
    return np.argwhere(np.isclose((coordinates - offset) % multiple, 0, atol = tolerance) | np.isclose((coordinates - offset) % multiple, multiple, atol = tolerance))

if __name__ == "__main__":
    main()
