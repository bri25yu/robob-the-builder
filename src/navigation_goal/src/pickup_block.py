#!/usr/bin/env python

import os
import sys
sys.path.insert(0, "/home/aatifjiwani/Documents/BerkeleySenior/EECS106A/project_workspaces/robob-the-builder/src/world_simulation/src")
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Twist, Point
from std_msgs.msg import String, Duration, Float64
from moveit_commander.conversions import pose_to_list

from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from actionlib_msgs.msg import GoalID

from tf.transformations import quaternion_from_euler

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from std_srvs.srv import Empty
from navigation_goal.srv import GoalDirection

import numpy as np
import rosservice

from global_constants import utils
from navigation import move_to_goal, halt_robot
import world_simulation #import world_simulation

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
        self.cmd_pub = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size=10)

        self._initialize_frames()

        ## Services
        self.ng_pickup = rospy.Service("/ng_pickup", Empty, self.service_pickup)
        self.ng_place = rospy.Service("/ng_place", Empty, self.service_placedown)
        self.ng_prepare = rospy.Service("/ng_prepare", Empty, self.service_prepare)
        self.ng_init_pickup = rospy.Service("/ng_init_pickup", GoalDirection, self.initialize_pickup)

        # Subscribers

        self.corners = np.vstack(utils.get_corners("/home/aatifjiwani/Documents/BerkeleySenior/EECS106A/project_workspaces/robob-the-builder/output/schematic.txt"))
        self.corners_iter = iter(self.corners)
        print(self.corners)

        self.boxes_picked_up = 0
        self.cache_boxes = []

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

    def move_arm_to_states(self,):
        targets = [0.05, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]
        print("Attempting to move to ", targets)

        for joint_name, target_state in zip(self.joints, targets):
            print("Setting {} to {}".format(joint_name, target_state))
            self.group.set_joint_value_target(joint_name, target_state)

        self.group.set_planning_time(5.0)

        self.group.go(wait=True)
        self.group.stop()
        

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

    def add_planning_obstacle(self, name, x, y, z, h, w):
        rospy.sleep(2)
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_footprint"
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z
        box_pose.pose.orientation.w = 1

        self.scene.add_box(name, box_pose, size=(w,w,h))

    def add_map_obstacle(self, name, x, y, z, h, w):
        
        rospy.loginfo("Placing map obstacle for placed block")
        for box in self.cache_boxes:
            name, x, y, z, h, w = box
            rospy.sleep(1)
            box_pose = PoseStamped()
            box_pose.header.frame_id = "odom"
            box_pose.pose.position.x = x
            box_pose.pose.position.y = y
            box_pose.pose.position.z = z
            box_pose.pose.orientation.w = 1

            self.scene.add_box(name, box_pose, size=(w,w,h))
        
        rospy.loginfo("Placed")

    def add_cache_obstacles(self,):
        for box in self.cache_boxes:
            self.add_map_obstacle(*box)

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
        self.move_torso(0)

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

    

    def wait_for_aruco_detection(self, direction):
        while(True):
            try:
                rospy.wait_for_message('/aruco_single/pose', PoseStamped, timeout=4)
                return
            except:
                # turn the robot a little bit
                r = rospy.Rate(20) # 10hz
                move = Twist()
                move.angular.z = direction.data * np.pi/8
                for _ in range(20):
                    self.cmd_pub.publish(move)
                    r.sleep()


    def initialize_pickup(self, req):
        # Rotate the robot until we see a response from aruco_single/pose
        for block in self.cache_boxes:
            self.remove_obstacle(block[0])
            rospy.sleep(1)

        self.wait_for_aruco_detection(req.data)
        rospy.wait_for_service('/pick_gui')
        try:
            rospy.loginfo("Initialize pick service")
            pick_gui = rospy.ServiceProxy("/pick_gui", Empty)
            pick_gui()
        except rospy.ServiceException as e:
            print("Service call to prepare failed: %s" %e)

        return {}

    def back_robot_out(self,):
        r = rospy.Rate(20) # 10hz

        move = Twist()
        move.angular.z = -1
        for _ in range(25):
            self.cmd_pub.publish(move)
            r.sleep()

        rospy.sleep(2)
        move = Twist()
        move.linear.x = -0.5
        for _ in range(15):
            self.cmd_pub.publish(move)
            r.sleep()
        
    def service_pickup(self, req):
        self.move_gripper(0)
        self.move_torso(1)
        self.move_arm_to_pose(0.5, 0, 0.8, -np.pi/2, 0, np.pi/2)
        return {}

    def service_placedown(self, req):
        next_corner = next(self.corners_iter)
        next_corner[1] += 10

        print("Moving to NEW GOAL: ", next_corner)
        # world_simulation.worldsim_add_placeholder(Point(*list(next_corner)))

        move_to_goal(Point(0, 5, 0))
        rospy.sleep(3)
        move_to_goal(Point(*list(next_corner)))
        halt_robot()
        self.move_arm_to_pose(0.2, 0, 0.2, -np.pi/2, 0, 0)

        rospy.sleep(3)
        self.add_cache_obstacles()

        self.move_arm_to_pose(0.6, 0, 0.35, -np.pi/2, 0, np.pi/2)
        self.move_arm_to_pose(0.6, 0, 0.3, -np.pi/2, 0, np.pi/2)
        self.move_gripper(1)
        self.remove_obstacle("part")

        self.cache_boxes.append(["box_" + str(self.boxes_picked_up), next_corner[0] + 0.55, next_corner[1] - 0.05,  0.05, 0.43, 0.05])
        self.add_map_obstacle(*self.cache_boxes[-1])

        self.boxes_picked_up += 1
        self.back_robot_out()
        self.move_torso(0)

        self.prepare_robot()
        move_to_goal(Point(0, 5, 0))
        rospy.sleep(3)
        move_to_goal(Point(0, 0, 0))

        return {}

    def service_prepare(self, req):
        self.remove_obstacle("box")
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