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

    block_pose = Pose(position=Point(x=0.4225, y=-0.1265, z=0.7725))
    world_sim.add_block(block_pose, "world")

    block_pose = Pose(position=Point(x=1.4225, y=-0.1265, z=0.7725))
    world_sim.add_block(block_pose, "world")


class WorldSimulation:
    BLOCK_SIZE = (0.045, 0.045, 0.045)

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.moveit_scene = moveit_commander.PlanningSceneInterface()

        self.num_blocks = 0
        self.initialize_block_xml()

    def add_block(self, pose, reference_frame):
        """
        Adds a block to both Gazebo and RViz at pose based on the block_reference_frame

        Parameters
        ----------
        pose: Pose
            The pose of the block to place.
        reference_frame: str
            The name of the reference frame to place the block.

        """
        name = "block{0}".format(self.num_blocks)
        self.add_block_gazebo(pose, reference_frame, name)
        size = self.BLOCK_SIZE
        self.add_block_rviz(pose, reference_frame, name, "base_link", size)
        self.num_blocks += 1

    def remove_block(self, name):
        """
        Removes the block by the given name.

        Parameters
        ----------
        name : str
            The name of the block to remove.

        """
        self.remove_block_gazebo(name)
        self.remove_block_rviz(name, "base_link")
            
    #----------------------------------------------------------------------------------------------
    # Helper functions

    def initialize_block_xml(self, pack_name="world_simulation", model_dir="/models/", block_urdf_dir="block/block.urdf"):
        model_path = rospkg.RosPack().get_path(pack_name) + model_dir
        self.block_xml = ""
        with open (model_path + block_urdf_dir, "r") as block_file:
            self.block_xml = block_file.read().replace('\n', '')

    def add_block_gazebo(self, pose, reference_frame, name):
        rospy.wait_for_service(const.Gazebo.SPAWN_URDF_MODEL)
        try:
            spawn_urdf = rospy.ServiceProxy(const.Gazebo.SPAWN_URDF_MODEL, SpawnModel)
            resp_urdf = spawn_urdf(name, self.block_xml, "/", pose, reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    def add_block_rviz(self, pose, reference_frame, name, ref_link, size):
        self.moveit_scene.attach_box(
            ref_link,
            name,
            PoseStamped(Header(frame_id = reference_frame), pose),
            size,
        )

    def remove_block_gazebo(self, name):
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp_delete = delete_model(name)
        except rospy.ServiceException, e:
            print("Delete Model service call failed: {0}".format(e))

    def remove_block_rviz(self, name, ref_link):
        self.moveit_scene.remove_attached_object(ref_link, name)

if __name__ == "__main__":
    rospy.init_node('moveit_node')
    main()
