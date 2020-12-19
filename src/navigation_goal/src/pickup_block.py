#!/usr/bin/env python

import rospy
import moveit_commander
import numpy as np
import tf2_ros

from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import PoseStamped, Quaternion, Twist, Point
from std_msgs.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_geometry_msgs import do_transform_pose

from std_srvs.srv import Empty
from navigation_goal.srv import GoalDirection

from global_constants import utils,constants
from navigation import move_to_goal, halt_robot
import world_simulation

class Planner():
    def __init__(self,):

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm_torso"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.set_planner_id("SBLkConfigDefault")
        self.group.set_max_velocity_scaling_factor(0.2)

        ## Publishers
        self.gripper_pub = rospy.Publisher('gripper_controller/command', JointTrajectory, queue_size=10)
        self.torso_pub = rospy.Publisher('torso_controller/command', JointTrajectory, queue_size=10)
        self.arm_pub = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size=10)
        self.cmd_pub = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size=10)
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)

        self._initialize_frames()

        ## Services
        self.ng_pickup = rospy.Service("/ng_pickup", Empty, self.service_pickup)
        self.ng_place = rospy.Service("/ng_place", Empty, self.service_placedown)
        self.ng_prepare = rospy.Service("/ng_prepare", Empty, self.service_prepare)
        self.ng_init_pickup = rospy.Service("/ng_init_pickup", GoalDirection, self.initialize_pickup)

        SPACING = 0.18
        self.corners = np.vstack(utils.get_robob_corners(SPACING, "/home/aatifjiwani/Documents/BerkeleySenior/EECS106A/project_workspaces/robob-the-builder/output/schematic.txt"))
        self.corners_iter = iter(self.corners)

        self.corners_iter = iter(self.corners)

        self.tfBuffer = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tfBuffer)

        self.remove_obstacle("table")

    def _initialize_frames(self,):

        self.planning_frame = self.group.get_planning_frame()
        print("Reference frame: {}".format(self.planning_frame))

        self.end_effector_link = self.group.get_end_effector_link()
        print("End effector link: {}".format(self.end_effector_link))

        self.joints = self.group.get_joints()[:-1]
        print("Joint names: ", self.joints)

    def move_arm_to_pose(self, x, y, z, roll, pitch, yaw, planning_time=5):
        rospy.sleep(2)
        self.group.set_start_state_to_current_state()

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_footprint"
        pose_goal.pose.orientation = Quaternion(*quaternion_from_euler(roll, pitch, yaw))
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z

        print("Attempting to move to ", x, y, z, roll, pitch, yaw)

        self.group.set_planning_time(planning_time)
        self.group.set_pose_target(pose_goal)

        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def add_box_to_scene(self, name, pos_x, pos_y, pos_z, d, w, h):
        rospy.sleep(2)
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_footprint"
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z
        box_pose.pose.orientation.w = 1

        self.scene.add_box(name, box_pose, size=(d,w,h))

    def add_planning_obstacle(self, name, x, y, z, h, w):
        self.add_box_to_scene(name, x, y, z, w, w, h)

    def add_table_obstacle(self, goal_z):
        #define a virtual table below the object
        self.add_box_to_scene("table", 0.5, 0, 0, 1, 1.8, goal_z - 0.13)

    def remove_obstacle(self, name):
        rospy.sleep(2)
        self.scene.remove_world_object(name=name)

    def move_with_joint_trajectory(self, joint_names, positions, efforts, durations, publisher):
        rospy.sleep(2)

        traj = JointTrajectory()
        traj.header.frame_id = "base_footprint"
        traj.joint_names.extend(joint_names)

        traj_points = []
        for i, (p, d) in enumerate(zip(positions, durations)):
            traj_pt = JointTrajectoryPoint()
            traj_pt.positions = p
            traj_pt.time_from_start.secs = d

            if efforts is not None:
                traj_pt.effort = efforts[i]

            traj_points.append(traj_pt)

        traj.points = traj_points
        publisher.publish(traj)
            

    def move_gripper(self, theta):
        self.move_with_joint_trajectory(
            joint_names=["gripper_left_finger_joint", "gripper_right_finger_joint"],
            positions=[[theta, theta]],
            efforts=[[10,10]]
            durations = [1],
            publisher=self.gripper_pub
        )

    def move_head(self, theta=-0.75):
        self.move_with_joint_trajectory(
            joint_names=['head_1_joint', 'head_2_joint'],
            positions=[[0.0, theta]],
            efforts=None,
            durations=[2.0],
            publisher=self.head_pub
        )

    def move_torso(self, theta):
        self.move_with_joint_trajectory(
            joint_names=["torso_lift_joint"],
            positions=[[theta]],
            efforts=None,
            durations=[2],
            publisher=self.torso_pub
        )

    def prepare_robot(self,):
        self.move_torso(0)
        self.move_with_joint_trajectory(
            joint_names=["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"],
            positions=constants.PICKUP_TRAJECTORY,
            efforts=None,
            durations=constants.PICKUP_TIMES,
            publisher=self.arm_pub
        )

    def wait_for_aruco_detection(self, direction):
        while(True):
            try:
                rospy.wait_for_message('/aruco_single/pose', PoseStamped, timeout=4)
                return
            except:
                # turn the robot a little bit
                self.move_robot_base(0, direction.data * np.pi/8, 20)


    def initialize_pickup(self, req):
        # Rotate the robot until we see a response from aruco_single/pose
        self.wait_for_aruco_detection(req.data)
        rospy.wait_for_service('/pick_gui')
        try:
            rospy.loginfo("Initialize pick service")
            pick_gui = rospy.ServiceProxy("/pick_gui", Empty)
            pick_gui()
        except rospy.ServiceException as e:
            print("Service call to prepare failed: %s" %e)

        return {}

    def move_robot_base(self, linear_x, angular_z, time):
        r = rospy.Rate(20) # 10hz

        rospy.sleep(2)
        move = Twist()

        move.linear.x = linear_x
        move.angular.z = angular_z
        for _ in range(time):
            self.cmd_pub.publish(move)
            r.sleep()

    def back_robot_out(self,):
        self.move_robot_base(-0.1, 0, 8)
        self.move_robot_base(0, -0.1, 10)
        self.move_robot_base(-0.5, 0, 10)

    def service_pickup(self, req):
        self.move_gripper(0)
        self.move_torso(1)
        self.move_arm_to_pose(0.5, -0.5, 0.7, -np.pi/2, 0, 0, planning_time=10)
        return {}

    def service_placedown(self, req):
        # find next goal
        next_corner = next(self.corners_iter)
        next_corner[1] += 10

        print("Moving to NEW GOAL: ", next_corner)
        world_simulation.worldsim_add_placeholder(Point(*list(next_corner)))

        # move robot to goal
        move_to_goal(Point(0, 5, 0), set_angle=True)
        rospy.sleep(3)
        move_to_goal(Point(next_corner[0] + 0.01, next_corner[1]-0.19, 0))
        halt_robot()

        # move robot to preparation phase
        self.move_arm_to_pose(0.5, -0.5, 1, -np.pi/2, 0, 0, planning_time=10)
        self.move_head(-1)
        self.scene.remove_attached_object("arm_tool_link")

        # find the signal_aruco/pose and place the block relative to that pose
        signal_pose = rospy.wait_for_message('/signal_aruco/pose', PoseStamped, timeout=20)
        if signal_pose.header.frame_id[0] == "/":
            signal_pose.header.frame_id = signal_pose.header.frame_id[1:]

        tf_signal_pose = self.transform_aruco_pose(signal_pose) # aruco marker pose w.r.t. base_footprint frame
        goal_x = tf_signal_pose.pose.position.x
        goal_y = tf_signal_pose.pose.position.y
        goal_z = 0.35

        # move block to aruco pose
        rospy.sleep(3)
        self.move_arm_to_pose(goal_x - 0.03, goal_y - 0.15, goal_z, -np.pi/2, 0, np.pi/2, planning_time=10)
        self.move_arm_to_pose(goal_x - 0.035, goal_y - 0.15, goal_z - 0.07, -np.pi/2, 0, np.pi/2, planning_time=10)
        self.move_gripper(1)
        self.move_arm_to_pose(goal_x - 0.06, goal_y - 0.16, goal_z - 0.1, -np.pi/2, 0, np.pi/2, planning_time=10)

        # prepare to back out
        self.remove_obstacle("part")
        self.back_robot_out()
        self.remove_obstacle("table")
        self.move_torso(0)

        # reset
        self.prepare_robot()
        self.move_head(-0.75)
        move_to_goal(Point(0, 5, 0), set_angle=True, returning_to_pickup=True)
        rospy.sleep(3)
        move_to_goal(Point(0, 0, 0), set_angle=True, returning_to_pickup=True)

        return {}

    def service_prepare(self, req):
        self.remove_obstacle("box")
        self.prepare_robot()
        return {}

    def transform_aruco_pose(self, aruco_pose):
        ps = PoseStamped()
        ps.pose.position = aruco_pose.pose.position
        ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)
        ps.header.frame_id = aruco_pose.header.frame_id
        transform_ok = False

        while not transform_ok:
            try:
                transform = self.tfBuffer.lookup_transform("base_footprint",
                                       ps.header.frame_id,
                                       rospy.Time(0))
                aruco_ps = do_transform_pose(ps, transform)
                transform_ok = True
            except tf2_ros.ExtrapolationException as e:
                rospy.logwarn(
                    "Exception on transforming point... trying again \n(" +
                    str(e) + ")")
                rospy.sleep(0.01)
                ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)

        return aruco_ps

if __name__ == "__main__":
    rospy.init_node('tiago_joint_moveit')
    planner = Planner()
    rospy.spin()
