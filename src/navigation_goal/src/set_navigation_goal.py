#!/usr/bin/env python
import sys

import numpy as np

import rospy
import rospkg

import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from navigation_goal.srv import NavGoal


def main():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0.5
    goal.target_pose.pose.position.y = 1
    goal.target_pose.pose.orientation.w = 1.0
    
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def move_to_goal(goal_position):
    print("move_to_goal is called with" + str(goal_position.x) + " " + str(goal_position.y))
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    print("does it get here")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = goal.position
    goal.target_pose.pose.orientation.w = 1.0
    
    client.send_goal(goal)
    wait = client.wait_for_result()

    #TODO: handle locations we can't get to and end
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def get_nav_goals():
    
    rospy.wait_for_service('det_nav_goal', timeout=120)
    det_nav_goal = rospy.ServiceProxy('det_nav_goal', NavGoal, persistent=True)
    # TODO: do a spin before moving at all

    done = False
    while (not done):
        try:
            print("hello1")
            goal_position = det_nav_goal()
            print("hello2")
            move_to_goal(goal_position.position)
            print("hello3")
        except rospy.ServiceException as exc:
            print("Exploration complete.")
            done = True
            # We're done when det_nav_goal shuts down
            # print("det_nav_goal service did not process request: " + str(exc))

    det_nav_goal.close()
        
if __name__ == "__main__":
    rospy.init_node('navigation_goal_node')
    get_nav_goals()