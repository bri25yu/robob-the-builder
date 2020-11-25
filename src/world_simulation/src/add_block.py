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
    Quaternion,
)
from std_msgs.msg import Header

import moveit_commander

import constants as const
import utils as utils


def main():
    world_sim = WorldSimulation()
    # create_exploration_world(world_sim)
    create_world_with_structure(world_sim)


def create_exploration_world(world_sim):
    block_positions = [
        [-3, 3, 0],
        [-1, -4, 0],
        [3, -2, 0],
        [-4, -1, 0],
        [-1,-3, 0],
        [2, 4, 0],
        [1, -4, 0],
        [3, 0, 0],
        [-1, 4, 0],
        [1, -1, 0]
    ]
    for pos in block_positions:
        pose = Pose(position = Point(pos[0], pos[1], pos[2]))
        world_sim.add_block(pose)


def create_world_with_structure(world_sim):
    x, y = 2, 0
    side_lengths = [3, 1]  # must be in descending order
    for i, side_length in enumerate(side_lengths):
        world_sim.add_square_2d(x, y, side_length, i * world_sim.block_size[2])

    #top down and side views
    world_sim.add_camera((x, y, 1), (0, np.pi / 2, 0), "camera0")  # top down camera
    world_sim.add_camera((x - 1, y, 0), (0, 0, 0), "camera1")  # side view camera
    #angled views
    world_sim.add_camera((2.3, -.75, 1), (.2, .7, 1.55), "camera2")
    world_sim.add_camera((3, -.5, .5), (0, .2, 2.3), "camera3")

class WorldSimulation:
    BLOCK_DEFAULT_COLOR = "<material>Gazebo/Red</material>"
    BLOCK_COLOR_TEMPLATE = "<material>Gazebo/{}</material>"
    BLOCK_SIZE_START_FLAG = "<box size="
    BLOCK_SIZE_END_FLAG = "/>"
    BLOCK_INERTIA_DEFAULT = '<inertia  ixx="0.0" ixy="0.0"  ixz="0.0"  iyy="0.0"  iyz="0.0"  izz="0.0" />'
    BLOCK_INERTIA_TEMPLATE = '<inertia  ixx="{}" ixy="0.0"  ixz="0.0"  iyy="{}"  iyz="0.0"  izz="{}" />'
    BLOCK_MASS_START_FLAG = "<mass value="
    BLOCK_MASS_END_FLAG = "/>"
    BLOCK_DEFAULT_ORIGIN = '<origin xyz="0.0 0.0 0.0" />'
    BLOCK_ORIGIN_TEMPLATE = '<origin xyz="{} {} {}" />'
    CAMERA_DEFAULT_NAME = "{INPUT_CAMERA_NAME}"

    PACK_NAME = "world_simulation"
    MODEL_DIR = "/models/"

    BLOCK_URDF_PATH = "block/block.urdf"
    CAMERA_SDF_PATH = "kinect/model.sdf"

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.moveit_scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        #TODO: later, once we add gmapping, we should be able to set both reference frames to map
        self.gazebo_reference_frame = "world"
        self.rviz_reference_frame = self.robot.get_planning_frame()
        self.num_blocks = 0
        self.initialize_block_xml()
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
            x = start_x + i * self.block_size[0]
            y = start_y + (j - (side_length // 2)) * self.block_size[1]
            z = z
            pose = Pose(position=Point(x, y, z))

            color = c.value

            self.add_block(pose, color=color)

    def add_camera(self, position, orientation, name):
        """
        Parameters
        ----------
        position: list
            A list of floats [x, y, z] describing the center of the camera.
        orientation: list
            A list of float [roll, pitch, yaw] describing the orientation of the camera.

        """
        position = Point(x=position[0], y=position[1], z=position[2])
        x, y, z, w = utils.rpy_to_quaternion(*orientation)
        orientation = Quaternion(x=x, y=y, z=z, w=w)
        pose = Pose(position=position, orientation=orientation)

        camera_xml = self.camera_xml.replace(self.CAMERA_DEFAULT_NAME, name)

        rospy.wait_for_service(const.Gazebo.SPAWN_SDF_MODEL)
        try:
            spawn_sdf = rospy.ServiceProxy(const.Gazebo.SPAWN_SDF_MODEL, SpawnModel)
            resp_sdf = spawn_sdf(name, camera_xml, "/", pose, self.gazebo_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))

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

    def initialize_block_xml(self):
        model_path = rospkg.RosPack().get_path(self.PACK_NAME) + self.MODEL_DIR
        self.block_xml = ""
        with open (model_path + self.BLOCK_URDF_PATH, "r") as block_file:
            self.block_xml = block_file.read().replace("\n", "")

        self.initialize_block_size()
        self.initialize_block_inertia()

    def initialize_camera_xml(self):
        model_path = rospkg.RosPack().get_path(self.PACK_NAME) + self.MODEL_DIR
        self.camera_xml = ""
        with open (model_path + self.CAMERA_SDF_PATH, "r") as camera_file:
            self.camera_xml = camera_file.read().replace("\n", "")

    def initialize_block_size(self):
        block_size_str = utils.get_content_between(self.block_xml, self.BLOCK_SIZE_START_FLAG, self.BLOCK_SIZE_END_FLAG)
        block_size_str = block_size_str.replace('"', "").split()
        self.block_size = tuple(map(lambda s: float(s), block_size_str))

        self.block_origin = tuple([v / 2.0 for v in self.block_size])
        block_origin_str = self.BLOCK_ORIGIN_TEMPLATE.format(*self.block_origin)
        self.block_xml = self.block_xml.replace(self.BLOCK_DEFAULT_ORIGIN, block_origin_str)

        block_mass_str = utils.get_content_between(self.block_xml, self.BLOCK_MASS_START_FLAG, self.BLOCK_MASS_END_FLAG)
        block_mass_str = block_mass_str.replace('"', "").split()
        self.block_mass = float(block_mass_str[0])

    def initialize_block_inertia(self):
        x, y, z = self.block_size
        ixx = (1.0 / 12) * self.block_mass * (y*y + z*z)
        iyy = (1.0 / 12) * self.block_mass * (x*x + z*z)
        izz = (1.0 / 12) * self.block_mass * (x*x + y*y)
        self.block_inertia = (ixx, iyy, izz)
        block_inertia_str = self.BLOCK_INERTIA_TEMPLATE.format(*self.block_inertia)
        self.block_xml = self.block_xml.replace(self.BLOCK_INERTIA_DEFAULT, block_inertia_str)

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
        adjusted_x = pose.position.x + (size[0] / 2)
        adjusted_y = pose.position.y + (size[1] / 2)
        adjusted_z = pose.position.z + (size[2] / 2)
        adjusted_position = Point(x=adjusted_x, y=adjusted_y, z=adjusted_z)
        adjusted_pose = Pose(position=adjusted_position, orientation=pose.orientation)

        pose_stamped = PoseStamped(Header(frame_id = reference_frame), adjusted_pose)
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
