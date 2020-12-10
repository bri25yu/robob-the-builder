#!/usr/bin/env python
import sys

from itertools import product, cycle

import rospy
import rospkg

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
)
from std_msgs.msg import Header

import moveit_commander

import constants as const
from global_constants import constants as gconst, camera
import utils
import take_photos


def main():
    world_sim = WorldSimulation(gazebo_only=True, structure = False)
    create_exploration_world(world_sim)
    # world_sim = WorldSimulation(gazebo_only=True, structure = True)
    # create_world_with_structure(world_sim)


def create_exploration_world(world_sim):
    for pos in gconst.EXPLORATION_BLOCKS:
        world_sim.add_block(Pose(position=Point(*pos)))

    worldsim_add_signal_block(Point(3, 10, 0))

def worldsim_add_placeholder(position):
    world_sim = WorldSimulation(gazebo_only=True, structure = False)
    position.y += 0.125
    position.x += 0.55
    world_sim.add_placeholder(Pose(position=position))

def worldsim_add_signal_block(position):
    world_sim = WorldSimulation(gazebo_only=True, structure = False)
    world_sim.add_signal_block(Pose(position=position))

def create_world_with_structure(world_sim):
    for cam in camera.CAMERAS.items():
        world_sim.add_camera(*cam)
    for square in gconst.STRUCTURE_TO_BUILD:
        square = list(square)
        square[-1] *= world_sim.block_size[2]
        world_sim.add_square_2d(*square)
    take_photos.main()

class WorldSimulation:
    PACK_NAME = "world_simulation"
    MODEL_DIR = "/models/"

    def __init__(self, gazebo_only=False, structure = False):
        self.gazebo_only = gazebo_only
        moveit_commander.roscpp_initialize(sys.argv)
        self.moveit_scene = moveit_commander.PlanningSceneInterface()
        self.gazebo_reference_frame = "world"
        if not self.gazebo_only:
            self.robot = moveit_commander.RobotCommander()
            self.rviz_reference_frame = self.robot.get_planning_frame()
        self.num_blocks = 0
        self.initialize_block_xml(structure = structure)
        if structure is False:
            self.initialize_placeholder_xml()
            self.initialize_signal_block_xml()
        self.initialize_camera_xml()
        self.initialize_add_block()

    def add_square_2d(self, start_x, start_y, side_length, z=0.0):
        """
        Creates a square of blocks centered around (start_x, start_y).

        Parameters
        ----------
        start_x: float
            The x-coordinate of the center of the square of blocks.
        start_y: float
            The y-coordinate of the center of the square of blocks.
        side_length: int
            The number of blocks in the side length of the square.
        z: float
            The z-coordinate of all the blocks.

        """
        indices = list(range(side_length))
        color_cycle = self.colors

        for (i, j), c in zip(product(indices, indices), color_cycle):
            x = start_x + (i - (side_length / 2)) * self.block_size[0]
            y = start_y + (j - (side_length / 2)) * self.block_size[1]
            z = z
            pose = Pose(position=Point(x, y, z))

            color = c.value

            self.add_block(pose, color=color)

    def add_camera(self, name, pose):
        """
        Parameters
        ----------
        pose: Pose

        """
        camera_xml = self.camera_xml.replace(const.CAMERA_DEFAULT_NAME, name)
        utils.add_block_gazebo(const.Gazebo.SPAWN_SDF_MODEL, camera_xml, pose, self.gazebo_reference_frame, name)

    def add_block(self, pose, color=None):
        """
        Adds a block to both Gazebo and RViz at pose based on the block_reference_frame

        Parameters
        ----------
        pose: Pose
            The pose of the block to place.
        color: str
            The color of the block.

        """
        name = "block{0}".format(self.num_blocks)
        self.add_block_gazebo(pose, self.gazebo_reference_frame, name, color=color)
        if not self.gazebo_only:
            size = self.block_size
            self.add_block_rviz(pose, self.rviz_reference_frame, name, size)
        self.num_blocks += 1

    def add_placeholder(self, pose):
        name = "placeholder"
        self.add_placeholder_gazebo(pose, self.gazebo_reference_frame, name)

    def add_signal_block(self, pose):
        name = "signal_block"
        self.add_signal_block_gazebo(pose, self.gazebo_reference_frame, name)

    def remove_all_blocks(self, total):
        for i in range(total):
            self.remove_block_rviz("block{0}".format(i))

    def remove_block_rviz(self, name):
        self.moveit_scene.remove_world_object(name)

    #----------------------------------------------------------------------------------------------
    # Helper functions

    def initialize_xml(self, model_path):
        model_dir_path = rospkg.RosPack().get_path(self.PACK_NAME) + self.MODEL_DIR
        xml = ""
        with open (model_dir_path + model_path, "r") as xml_file:
            xml = xml_file.read().replace("\n", "")
        return xml

    def initialize_block_xml(self, structure = False):
        if structure:
            self.block_xml = self.initialize_xml(const.STRUCTURE_BLOCK_URDF_PATH)
        else:
            self.block_xml = self.initialize_xml(const.BLOCK_URDF_PATH)
        self.initialize_block_size()
        self.initialize_block_inertia()

    def initialize_placeholder_xml(self):
        self.placeholder_xml = self.initialize_xml(const.PLACEHOLDER_URDF_PATH)

    def initialize_signal_block_xml(self):
        self.signal_block_xml = self.initialize_xml(const.SIGNAL_BLOCK_URDF_PATH)

    def initialize_camera_xml(self):
        self.camera_xml = self.initialize_xml(const.CAMERA_SDF_PATH)

    def initialize_block_size(self):
        block_size_str = utils.get_content_between(self.block_xml, const.Block.Size.START_FLAG, const.Block.Size.END_FLAG)
        block_size_str = block_size_str.replace('"', "").split()
        self.block_size = tuple(map(lambda s: float(s), block_size_str))

        self.block_origin = tuple([v / 2.0 for v in self.block_size])
        block_origin_str = const.Block.Origin.TEMPLATE.format(*self.block_origin)
        self.block_xml = self.block_xml.replace(const.Block.Origin.DEFAULT, block_origin_str)

        block_mass_str = utils.get_content_between(self.block_xml, const.Block.Mass.START_FLAG, const.Block.Mass.END_FLAG)
        block_mass_str = block_mass_str.replace('"', "").split()
        self.block_mass = float(block_mass_str[0])

    def initialize_block_inertia(self):
        x, y, z = self.block_size
        ixx = (1.0 / 12) * self.block_mass * (y*y + z*z)
        iyy = (1.0 / 12) * self.block_mass * (x*x + z*z)
        izz = (1.0 / 12) * self.block_mass * (x*x + y*y)
        self.block_inertia = (ixx, iyy, izz)

        block_inertia_str = const.Block.Inertia.TEMPLATE.format(*self.block_inertia)
        self.block_xml = self.block_xml.replace(const.Block.Inertia.DEFAULT, block_inertia_str)

    def initialize_add_block(self):
        self.colors = cycle(const.Colors)

    def add_block_gazebo(self, pose, reference_frame, name, color=None):
        colored_xml = self.block_xml
        if color is not None:
            colored_xml = colored_xml.replace(const.Block.Color.DEFAULT, const.Block.Color.TEMPLATE.format(color))
        utils.add_block_gazebo(const.Gazebo.SPAWN_URDF_MODEL, colored_xml, pose, reference_frame, name)

    def add_placeholder_gazebo(self, pose, reference_frame, name):
        utils.add_block_gazebo(const.Gazebo.SPAWN_URDF_MODEL, self.placeholder_xml, pose, reference_frame, name)

    def add_signal_block_gazebo(self, pose, reference_frame, name):
        utils.add_block_gazebo(const.Gazebo.SPAWN_URDF_MODEL, self.signal_block_xml, pose, reference_frame, name)

    def add_block_rviz(self, pose, reference_frame, name, size):
        adjusted_x = pose.position.x + (size[0] / 2)
        adjusted_y = pose.position.y + (size[1] / 2)
        adjusted_z = pose.position.z + (size[2] / 2)
        adjusted_position = Point(x=adjusted_x, y=adjusted_y, z=adjusted_z)
        adjusted_pose = Pose(position=adjusted_position, orientation=pose.orientation)

        pose_stamped = PoseStamped(Header(frame_id = reference_frame), adjusted_pose)
        self.moveit_scene.add_box(name, pose_stamped, size)


if __name__ == "__main__":
    rospy.init_node('moveit_node')
    main()
