#!/usr/bin/env python
import sys

from itertools import product, cycle
import numpy as np

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
)
from std_msgs.msg import Header

import moveit_commander

import constants as const


def main():
    world_sim = WorldSimulation()

    world_sim.add_square_2d(2, 0, 5)


class WorldSimulation:
    BLOCK_DEFAULT_COLOR = "<material>Gazebo/Red</material>"
    BLOCK_COLOR_TEMPLATE = "<material>Gazebo/{}</material>"
    BLOCK_SIZE_START_FLAG = "<box size="
    BLOCK_SIZE_END_FLAG = "/>"

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.moveit_scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        #TODO: later, once we add gmapping, we should be able to set both reference frames to map
        self.gazebo_reference_frame = "world"
        self.rviz_reference_frame = self.robot.get_planning_frame()
        self.num_blocks = 0
        self.initialize_block_xml()
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
            x = start_x + i * block_size
            y = start_y + (j - (side_length // 2)) * block_size
            z = z
            pose = Pose(position=Point(x, y, z))

            color = c.value

            self.add_block(pose, color=color)

    def remove_all_blocks(self, total):
        for i in range(total):
            self.remove_block("block{0}".format(i))

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
        size = self.block_size
        self.add_block_rviz(pose, self.rviz_reference_frame, name, size)
        self.num_blocks += 1

    def remove_block(self, name):
        """
        Removes the block by the given name.

        Parameters
        ----------
        name : str
            The name of the block to remove.

        """
        # self.remove_block_gazebo(name)
        self.remove_block_rviz(name)

    #----------------------------------------------------------------------------------------------
    # Helper functions

    def initialize_block_xml(self, pack_name="world_simulation", model_dir="/models/", block_urdf_dir="block/block.urdf"):
        model_path = rospkg.RosPack().get_path(pack_name) + model_dir
        self.block_xml = ""
        with open (model_path + block_urdf_dir, "r") as block_file:
            self.block_xml = block_file.read().replace('\n', '')

        start_i = self.block_xml.find(self.BLOCK_SIZE_START_FLAG)
        end_i = self.block_xml.find(self.BLOCK_SIZE_END_FLAG, start_i)
        block_size_str = self.block_xml[start_i + len(self.BLOCK_SIZE_START_FLAG): end_i].replace('"', "").split()
        self.block_size = tuple(map(lambda s: float(s), block_size_str))

    def initialize_add_block(self):
        self.colors = cycle(const.Colors)

    def add_block_gazebo(self, pose, reference_frame, name, color=None):
        colored_xml = self.block_xml
        if color is not None:
            colored_xml = self.block_xml.replace(self.BLOCK_DEFAULT_COLOR, self.BLOCK_COLOR_TEMPLATE.format(color))

        rospy.wait_for_service(const.Gazebo.SPAWN_URDF_MODEL)
        try:
            spawn_urdf = rospy.ServiceProxy(const.Gazebo.SPAWN_URDF_MODEL, SpawnModel)
            resp_urdf = spawn_urdf(name, colored_xml, "/", pose, reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    def add_block_rviz(self, pose, reference_frame, name, size):
        #rospy.sleep(2)
        pose_stamped = PoseStamped(Header(frame_id = reference_frame), pose)
        self.moveit_scene.add_box(name, pose_stamped, size)


    def remove_block_gazebo(self, name):
        try:
            delete_model = rospy.ServiceProxy(const.Gazebo.DELETE_MODEL, DeleteModel)
            resp_delete = delete_model(str(name))
        except Exception, e:
            print("Delete Model service call failed: {0}".format(e))

    def remove_block_rviz(self, name):
        self.moveit_scene.remove_world_object(name)

if __name__ == "__main__":
    rospy.init_node('moveit_node')
    main()
