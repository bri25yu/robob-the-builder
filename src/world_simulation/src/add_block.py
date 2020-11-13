#!/usr/bin/env python
import sys

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

    for d in const.TEST_STRUCTURE:
        pose, color = d[const.El.POSE], d[const.El.COLOR]
        block_pose = Pose(position=Point(**pose))
        world_sim.add_block(block_pose, color=color)
    # world_sim.remove_all_blocks(3)


class WorldSimulation:
    BLOCK_SIZE = (0.045, 0.045, 0.045)
    BLOCK_DEFAULT_COLOR = "<material>Gazebo/Red</material>"
    BLOCK_COLOR_TEMPLATE = "<material>Gazebo/{}</material>"

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.moveit_scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        #TODO: later, once we add gmapping, we should be able to set both reference frames to map
        self.gazebo_reference_frame = "world"
        self.rviz_reference_frame = self.robot.get_planning_frame()
        self.num_blocks = 0
        self.initialize_block_xml()

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
        size = self.BLOCK_SIZE
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
    #TODO: add support for different colors
    def initialize_block_xml(self, pack_name="world_simulation", model_dir="/models/", block_urdf_dir="block/block.urdf"):
        model_path = rospkg.RosPack().get_path(pack_name) + model_dir
        self.block_xml = ""
        with open (model_path + block_urdf_dir, "r") as block_file:
            self.block_xml = block_file.read().replace('\n', '')

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
