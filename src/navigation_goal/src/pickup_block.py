#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from tf.transformations import quaternion_from_euler

class Planner():
    def __init__(self,):

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        group_name = "arm_torso"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.set_planner_id("SBLkConfigDefault")

        ## trajectory publisher. we may or may not need this in the future
        self.traj_pub = rospy.Publisher('tiago_move_group/display_planned_path', DisplayTrajectory, queue_size=20)

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

def main():
    planner = Planner()
    # planner.move_arm_to_states(None)
    planner.move_arm_to_pose(0.636252238837, 0.0625501013898, 0.354975569372, 0, 0, 0.0)

if __name__ == "__main__":
    rospy.init_node('tiago_joint_moveit')
    main()