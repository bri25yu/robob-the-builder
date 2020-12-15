#!/usr/bin/env python

from itertools import product

import rospy
import cv2
import numpy as np

from feature_detect import FeatureDetect
from global_constants.camera import CameraDTO
from image_matching import ImageMatching
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from global_constants import utils as gutils, constants as gconst


Z_DIFF_3D = np.array([0, 0, gconst.BLOCK_Z])

def main():
    # gs = GenerateSchematic()
    # gs.process()
    # gs.display()

    gss = GenerateSchematicStats()
    gss.num_pairs_of_cameras_test()


class GenerateSchematic:
    def __init__(self):
        self.initialize()

    def initialize(self):
        self.raw_world_coordinates = None
        self.processed_world_coordinates = None
        self.world_coordinates = None
        self.bottom_left_corners = None

    def process(self, camera_pair_indices=None, output_corners=True):
        """
        Parameters
        ----------
        camera_pair_indices: A (c, 2)-shaped np.ndarray

        """
        if camera_pair_indices is None:
            camera_pair_indices = GenerateSchematic.generate_all_camera_pair_indices()

        self.raw_world_coordinates = GenerateSchematic.get_raw_world_coordinates(camera_pair_indices)
        if self.raw_world_coordinates is None: return

        self.processed_world_coordinates = GenerateSchematic.process_raw_world_coordinates(self.raw_world_coordinates)
        if self.processed_world_coordinates is None: return

        self.world_coordinates = GenerateSchematic.push_down(self.processed_world_coordinates)
        if self.world_coordinates is None: return

        self.bottom_left_corners = GenerateSchematic.output_bottom_left_corners(self.world_coordinates, output_corners=output_corners)

    @staticmethod
    def get_raw_world_coordinates(camera_pair_indices):
        """
        Assumption 1: blocks are rectangular prisms
        """
        raw_world_coordinates = []
        for i1, i2 in camera_pair_indices:
            raw_pair_coordinates = GenerateSchematic.get_raw_world_coordinates_for_pair(i1, i2)
            if len(raw_pair_coordinates) > 0:
                raw_world_coordinates.append(raw_pair_coordinates)

        raw_world_coordinates = np.vstack(raw_world_coordinates)

        return raw_world_coordinates

    @staticmethod
    def process_raw_world_coordinates(raw_world_coordinates):
        """
        Assumption 1: blocks are the same size
        Assumption 2: blocks are strictly aligned to multiples of their sizes, given their offset
        Assumption 3: block orientations are axis-aligned
        Assumption 4: all blocks have the same orientation
        Assumption 5: we know the blocks' orientation
        """
        # We first take only the coordinates that are grid aligned
        potential_grid_indices, max_length, max_offset = [], float("-inf"), None
        x_offsets_to_try = np.arange(0, gconst.BLOCK_X, step=0.01)
        y_offset_to_try = np.arange(0, gconst.BLOCK_Y, step=0.01)
        for x_o, y_o in product(x_offsets_to_try, y_offset_to_try):
            offset = np.array([x_o, y_o, 0])
            indices = gutils.close_to_multiples_of(raw_world_coordinates, gconst.multiple, offset, gconst.tolerances)
            if len(indices) > max_length:
                potential_grid_indices, max_length, max_offset = indices, len(indices), offset

        grid_aligned = raw_world_coordinates[potential_grid_indices]

        # Then, we round all of our coordinates
        rounded = gutils.round_nearest(grid_aligned, max_offset, gconst.multiple)

        # Finally, we removed all duplicate coordinates
        processed_world_coordinates = gutils.unique_rows(rounded)

        return processed_world_coordinates

    @staticmethod
    def push_down(processed_world_coordinates):
        """
        Assumption 1: blocks are the same size
        Assumption 2: block orientations are axis-aligned
        Assumption 3: all blocks have the same orientation
        Assumption 4: we know the blocks' orientation
        """
        # Go downward layer by layer
        # At each layer remove non-squares and project points to lower layers
        layers = gutils.get_layers(processed_world_coordinates)
        filtered_layers = []
        for i in reversed(range(len(layers))):
            cur_layer = GenerateSchematic.apply_square_filter_on_layer(layers[i][:, :2])
            if len(cur_layer) != 0:
                cur_layer = np.hstack((cur_layer, np.ones((len(cur_layer), 1)) * layers[i][0][2]))
                filtered_layers.append(cur_layer)
                if i > 0:
                    layers[i-1] = np.vstack((layers[i-1], cur_layer - Z_DIFF_3D))

        if len(filtered_layers) == 0: return None

        world_coordinates = np.vstack(filtered_layers)
        world_coordinates = gutils.unique_rows(world_coordinates)

        return world_coordinates

    @staticmethod
    def output_bottom_left_corners(world_coordinates, output_corners):
        """
        Assumption 1: all the blocks are the same height
        Assumption 2: all the blocks are in a convex 2D rectangle shape
        """
        layers_to_output = gutils.get_layers(world_coordinates)[1:]

        bottom_left_corners = np.vstack(GenerateSchematic.get_bottom_left_corners(layer) for layer in layers_to_output) - Z_DIFF_3D

        if output_corners:
            gutils.output_corners(bottom_left_corners)

            return gutils.get_corners()
        else:
            return bottom_left_corners

    def display(self):
        fig = plt.figure(figsize=plt.figaspect(0.5))
        ax1 = fig.add_subplot(2, 2, 1, projection='3d')
        ax2 = fig.add_subplot(2, 2, 2, projection='3d')
        ax3 = fig.add_subplot(2, 2, 3, projection='3d')
        ax4 = fig.add_subplot(2, 2, 4, projection='3d')

        title = "Raw world coordinates"
        ImageMatching.scatter3d(self.raw_world_coordinates, ax1, title=title)
        print(title, self.raw_world_coordinates)

        title = "World coords before filtering"
        ImageMatching.scatter3d(self.processed_world_coordinates, ax2, title=title)
        print(title, self.processed_world_coordinates)

        title = "World coords after processing"
        ImageMatching.scatter3d(self.world_coordinates, ax3, title=title)
        print(title, self.world_coordinates)

        title = "Bottom left corners sent to robot"
        ImageMatching.scatter3d(self.bottom_left_corners, ax4, title=title)
        print(title, self.bottom_left_corners)

        plt.savefig("output/world_coordinates.jpg")
        plt.show()

    # Helpers--------------------------------------------------------------------------------------

    @staticmethod
    def generate_all_camera_pair_indices():
        n = len(gconst.CAMERA_DATA)
        return np.reshape(np.arange(n), (n // 2, 2))

    @staticmethod
    def get_raw_world_coordinates_for_pair(n1, n2, epipolar_threshold=0.01):
        camera1, camera2 = CameraDTO(n1), CameraDTO(n2)
        camera2_coordinates = FeatureDetect.find_all_corners_3d(camera1, camera2, epipolar_threshold=epipolar_threshold)
        g02 = CameraDTO.get_g(camera2.pose)
        world_coordinates = ImageMatching.apply_transform(ImageMatching.lift(camera2_coordinates), g02)[:, :3]
        return world_coordinates

    @staticmethod
    def apply_square_filter_on_layer(coordinates):
        """
        Parameters
        ----------
        coordinates: (n, 2)-shaped np.ndarray

        """
        x_diff, y_diff = gconst.BLOCK_X, gconst.BLOCK_Y
        #remove points that aren't part of squares
        filtered_coordinates = []
        for coordinate in coordinates:
            x, y = coordinate[0], coordinate[1]
            possible_squares = [[(x + x_diff, y + y_diff), (x + x_diff, y), (x, y + y_diff)],
                                [(x - x_diff, y), (x - x_diff, y - y_diff), (x, y - y_diff)],
                                [(x + x_diff, y), (x, y - y_diff), (x + x_diff, y - y_diff)],
                                [(x - x_diff, y), (x, y + y_diff), (x - x_diff, y + y_diff)]]

            for square in possible_squares:
                square_corners = 0
                for i in range(3):
                    for c in coordinates:
                        if np.allclose(c, square[i]):
                            square_corners += 1
                            break
                if square_corners >= 3:
                    filtered_coordinates.append(coordinate)
                    break

        return np.array(filtered_coordinates)

    @staticmethod
    def get_bottom_left_corners(coordinates):
        """
        Bottom left is defined as the min(x), min(y), min(z).d

        Parameters
        ----------
        coordinates: (n, 3)-shaped np.ndarray

        Returns
        -------
        bottom_left_corners: np.ndarray

        """
        max_x, max_y = np.max(coordinates[:, 0]), np.max(coordinates[:, 1])
        return coordinates[np.ravel(np.argwhere(1 - (np.isclose(coordinates[:, 0], max_x) | np.isclose(coordinates[:, 1], max_y))))]


class GenerateSchematicStats:
    def __init__(self):
        pass

    def run_test(self):
        pass

    def num_pairs_of_cameras_test(self, num_itrs=5):
        total_indices = GenerateSchematic.generate_all_camera_pair_indices()
        num_indices = list(range(len(total_indices)))
        expected_output = gutils.get_corners()

        num_corners_correct = []
        num_spurious_corners = []
        num_pairs_of_cameras = list(range(1, len(total_indices) + 1))
        for i, s in enumerate(num_pairs_of_cameras):
            num_corners_correct.append(0.0)
            num_spurious_corners.append(0.0)

            for _ in range(num_itrs):
                gs = GenerateSchematic()

                gs.process(total_indices[np.random.choice(num_indices, s, replace=False)], output_corners=False)

                if gs.bottom_left_corners is None:
                    continue

                for corner in gs.bottom_left_corners:
                    if corner in expected_output:
                        num_corners_correct[-1] += 1
                    else: 
                        num_spurious_corners[-1] += 1

            num_corners_correct[-1] /= num_itrs * len(expected_output)
            num_spurious_corners[-1] /= num_itrs * len(expected_output)

            print("Finished {} / {}".format(i + 1, len(num_pairs_of_cameras)))
        
        fig, axs = plt.subplots(2)
        axs[0].plot(num_pairs_of_cameras, num_corners_correct)
        axs[0].set_xlabel("Number of pairs of cameras")
        axs[0].set_ylabel("Percent of corners correct")
        axs[0].set_title("Percent of corners correct vs number of pairs of cameras")
        axs[1].plot(num_pairs_of_cameras, num_spurious_corners)
        axs[1].set_xlabel("Number of pairs of cameras")
        axs[1].set_ylabel("Ratio of spurious matches to number of expected matches")
        axs[1].set_title("Spurious matches vs number of pairs of cameras")

        plt.savefig("output/num_pairs_of_cameras_test.jpg")
        plt.show()


if __name__ == "__main__":
    main()
