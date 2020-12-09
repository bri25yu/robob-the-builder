#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from std_msgs.msg import String, Duration
from moveit_commander.conversions import pose_to_list

from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from actionlib_msgs.msg import GoalID

from tf.transformations import quaternion_from_euler

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from std_srvs.srv import Empty

import numpy as np
import rosservice

class Planner():
    def __init__(self,):

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        group_name = "arm_torso"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.set_planner_id("SBLkConfigDefault")

        ## Publishers
        self.gripper_pub = rospy.Publisher('gripper_controller/command', JointTrajectory, queue_size=10)
        # self.gripper_pub = rospy.Publisher('gripper_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
        # self.gripper_stop_pub = rospy.Publisher('gripper_controller/follow_joint_trajectory/cancel', GoalID, queue_size=10)
        self.torso_pub = rospy.Publisher('torso_controller/command', JointTrajectory, queue_size=10)
        self.arm_pub = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size=10)

        self._initialize_frames()

        ## Services
        self.ng_pickup = rospy.Service("/ng_pickup", Empty, self.service_pickup)
        self.ng_place = rospy.Service("/ng_place", Empty, self.service_placedown)
        self.ng_prepare = rospy.Service("/ng_prepare", Empty, self.service_prepare)

        ## Extra
        # self.gripper_goal = FollowJointTrajectoryActionGoal()
        # self.gripper_goal.header.frame_id = "base_footprint"
        # self.gripper_goal.goal_id.id = "69"

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
        self.group.set_goal_position_tolerance(0.01)

        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def add_planning_obstacle(self, name, x, y, z, ox, oy, oz, ow, h, w):
        rospy.sleep(2)
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_footprint"
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z

        box_pose.pose.orientation.x = 0
        box_pose.pose.orientation.y = 0
        box_pose.pose.orientation.z = 0
        box_pose.pose.orientation.w = 1

        self.scene.add_box(name, box_pose, size=(w,w,h))

    def remove_obstacle(self, name):
        rospy.sleep(2)
        self.scene.remove_world_object(name=name)

    def move_gripper(self, theta):
        # rospy.sleep(2)
        # self.gripper_stop_pub.publish(self.gripper_goal.goal_id)

        rospy.sleep(2)

        gripper_traj = JointTrajectory()
        gripper_traj.header.frame_id = "base_footprint"
        gripper_traj.header.stamp.secs = rospy.Time.now().to_sec()
        gripper_traj.joint_names.extend(["gripper_left_finger_joint", "gripper_right_finger_joint"])

        duration = rospy.rostime.Duration()
        duration.secs = 5

        gripper_traj_point = JointTrajectoryPoint()
        gripper_traj_point.positions = [theta, theta]
        gripper_traj_point.effort = [10, 10]
        gripper_traj_point.time_from_start = duration

        gripper_traj.points = [gripper_traj_point]

        # self.gripper_goal.goal.trajectory = gripper_traj

        self.gripper_pub.publish(gripper_traj)
        # self.gripper_pub.publish(self.gripper_goal)
        # self.gripper_pub.publish()

    def move_torso(self, theta):
        rospy.sleep(2)

        torso_traj = JointTrajectory()
        torso_traj.header.frame_id = "base_footprint"
        torso_traj.joint_names.extend(["torso_lift_joint"])

        duration = rospy.rostime.Duration()
        duration.secs = 2

        torso_traj_point = JointTrajectoryPoint()
        torso_traj_point.positions = [theta]
        torso_traj_point.time_from_start = duration

        torso_traj.points = [torso_traj_point]

        self.torso_pub.publish(torso_traj)

    def prepare_robot(self,):
        self.move_torso(0.05)

        arm_traj = JointTrajectory()
        arm_traj.header.frame_id = "base_footprint"
        arm_traj.joint_names.extend(["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"])

        points = [
            [0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0],
            [0.21, -1.02, -0.20, 1.94, -1.57, 1.52, 0.0],
            [0.21, 0.35, -0.2, 2.0, -1.57, 1.52, 0.0],
            [0.21, 0.35, -0.2, 0.0, -1.57, 1.52, 0.0],
            [0.21, 0.35, -3.0, 0.0, -1.57, 1.52, 0.0],
            [0.05, -0.07, -3.0, 1.5, -1.57, 0.2, 0.0]
        ]

        times = [0, 1.5, 4.5, 8, 10, 15]

        traj_points = []
        for p, t in zip(points, times):
            arm_traj_point = JointTrajectoryPoint()
            arm_traj_point.positions = p
            arm_traj_point.time_from_start.secs = t

            traj_points.append(arm_traj_point)

        arm_traj.points = traj_points

        self.arm_pub.publish(arm_traj)

    def service_pickup(self, req):
        self.move_gripper(0)
        self.move_torso(0.75)
        return {}

    def service_placedown(self, req):
        self.move_arm_to_pose(x - 0.2, 0, 0.3, np.pi/2, 0, np.pi/2)
        self.move_gripper(1)
        return {}

    def service_prepare(self, req):
        self.prepare_robot()
        return {}

    

if __name__ == "__main__":
    rospy.init_node('tiago_joint_moveit')
    planner = Planner()
    rospy.spin()

"""
      x: 0.530766213999
      y: 0.0503877542722
      z: 0.34337055955


"""