#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from std_msgs.msg import String, Duration
from moveit_commander.conversions import pose_to_list

from tf.transformations import quaternion_from_euler

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np

class Planner():
    def __init__(self,):

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        group_name = "arm_torso"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.set_planner_id("SBLkConfigDefault")

        ## trajectory publisher. we may or may not need this in the future
        self.traj_pub = rospy.Publisher('tiago_move_group/display_planned_path', DisplayTrajectory, queue_size=20)

        self.gripper_pub = rospy.Publisher('gripper_controller/command', JointTrajectory, queue_size=10)

        self._initialize_frames()

    def _initialize_frames(self,):

        self.planning_frame = self.group.get_planning_frame()
        print("Reference frame: {}".format(self.planning_frame))


        self.end_effector_link = self.group.get_end_effector_link()
        print("End effector link: {}".format(self.end_effector_link))


        self.joints = self.group.get_joints()[:-1]
        print("Joint names: ", self.joints)

    def move_arm_to_states(self, joint_states = None):

        if joint_states is None:
            targets = [0, 2.0, 0.2, -2.1, 2.0, 1.0, -0.8, 0]
            print("Attempting to move to ", targets)

            for joint_name, target_state in zip(self.joints, targets):
                print("Setting {} to {}".format(joint_name, target_state))
                self.group.set_joint_value_target(joint_name, target_state)

            self.group.set_planning_time(5.0)

            self.group.go(wait=True)
            self.group.stop()
        else:
            raise NotImplementedError("need to add support for arbitrary joint states")

    def move_arm_to_pose(self, x, y, z, roll, pitch, yaw):
        rospy.sleep(2)
        self.group.set_start_state_to_current_state()

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_footprint"
        pose_goal.pose.orientation = Quaternion(*quaternion_from_euler(roll, pitch, yaw))
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z

        print("Attempting to move to ", x, y, z, roll, pitch, yaw)

        self.group.set_planning_time(5.0)
        self.group.set_pose_target(pose_goal)

        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def add_planning_obstacle(self, name, x, y, z, h, w):
        rospy.sleep(2)
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_footprint"
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z
        box_pose.pose.orientation.w = 1

        self.scene.add_box(name, box_pose, size=(w,w,h))

    def remove_obstacle(self, name):
        rospy.sleep(2)
        self.scene.remove_world_object(name=name)

    def move_gripper(self, theta):

        rospy.sleep(2)

        gripper_traj = JointTrajectory()
        gripper_traj.header.frame_id = "base_footprint"
        gripper_traj.joint_names.extend(["gripper_left_finger_joint", "gripper_right_finger_joint"])

        duration = rospy.rostime.Duration()
        duration.secs = 2

        gripper_traj_point = JointTrajectoryPoint()
        gripper_traj_point.positions = [theta, theta]
        gripper_traj_point.time_from_start = duration

        gripper_traj.points = [gripper_traj_point]

        self.gripper_pub.publish(gripper_traj)



def main():
    planner = Planner()

    x= 0.553933119422
    y= 0.0560869665656
    z= 0.33675939748

    x -= 0.01
    y += 0.02

    h = 0.4
    w = 0.03


    planner.move_gripper(0.5)
    # planner.move_arm_to_states(None)

    # planner.remove_obstacle("box")
    # planner.add_planning_obstacle("box", x, y, z, h, w)
    # planner.move_arm_to_pose(x, y - 0.05, z, -np.pi/2, 0, np.pi/2)

if __name__ == "__main__":
    rospy.init_node('tiago_joint_moveit')
    main()

"""
      x: 0.530766213999
      y: 0.0503877542722
      z: 0.34337055955


"""