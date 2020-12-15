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


Z_DIFF_3D = np.array([0, 0, gconst.BLOCK_Z])

def main():
    gs = GenerateSchematic()
    gs.process()
    gs.display()


class GenerateSchematic:
    def process(self):
        self.raw_world_coordinates = GenerateSchematic.get_raw_world_coordinates()
        self.processed_world_coordinates = GenerateSchematic.process_raw_world_coordinates(self.raw_world_coordinates)
        self.world_coordinates = GenerateSchematic.push_down(self.processed_world_coordinates)
        self.bottom_left_corners = GenerateSchematic.output_bottom_left_corners(self.world_coordinates)

    @staticmethod
    def get_raw_world_coordinates():
        """
        Assumption 1: blocks are rectangular prisms
        """
        raw_world_coordinates = []
        for i in range(len(gconst.CAMERA_DATA) // 2):
            raw_pair_coordinates = GenerateSchematic.get_raw_world_coordinates_for_pair(2 * i, 2 * i + 1)
            if len(raw_pair_coordinates) > 0:
                raw_world_coordinates.append(raw_pair_coordinates)

        raw_world_coordinates = np.vstack(raw_world_coordinates)

        return raw_world_coordinates

    @staticmethod
    def process_raw_world_coordinates(raw_world_coordinates):
        """
        Assumption 1: blocks are the same size
        Assumption 2: we know our offset
        Assumption 3: blocks are strictly aligned to multiples of their sizes, given their offset
        Assumption 4: block orientations are axis-aligned
        Assumption 5: all blocks have the same orientation
        Assumption 6: we know the blocks' orientation
        """
        # We first take only the coordinates that are grid aligned
        potential_grid_indices = gutils.close_to_multiples_of(raw_world_coordinates, gconst.multiple, gconst.offset, gconst.tolerances)
        grid_aligned = raw_world_coordinates[potential_grid_indices]

        # Then, we round all of our coordinates
        rounded = gutils.round_nearest(grid_aligned, gconst.offset, gconst.multiple)

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

        world_coordinates = np.vstack(filtered_layers)
        world_coordinates = gutils.unique_rows(world_coordinates)

        return world_coordinates

    @staticmethod
    def output_bottom_left_corners(world_coordinates):
        """
        Assumption 1: all the blocks are the same height
        Assumption 2: all the blocks are in a convex 2D rectangle shape
        """
        layers_to_output = gutils.get_layers(world_coordinates)[1:]

        bottom_left_corners = np.vstack(GenerateSchematic.get_bottom_left_corners(layer) for layer in layers_to_output) - Z_DIFF_3D

        gutils.output_corners(bottom_left_corners)

        return gutils.get_corners()

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
    def get_raw_world_coordinates_for_pair(n1, n2, epipolar_threshold=0.01):
        camera1, camera2 = CameraDTO(n1), CameraDTO(n2)
        camera2_coordinates = FeatureDetect.find_all_corners_3d(camera1, camera2, epipolar_threshold=epipolar_threshold).T
        g02 = CameraDTO.get_g(camera2.pose)
        camera2_coordinates = camera2_coordinates.T
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


if __name__ == "__main__":
    main()
