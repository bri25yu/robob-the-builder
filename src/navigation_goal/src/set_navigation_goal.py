#!/usr/bin/env python
import sys

import numpy as np

import rospy
import rospkg

import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


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

def hi():
    rospy.wait_for_service('det_nav_goal', timeout=120)
    det_nav_goal = rospy.ServiceProxy('det_nav_goal', Nav_goal, persistent=True)
    try:
       resp1 = add_two_ints(x, y)
    except rospy.ServiceException as exc:
      print("Service did not process request: " + str(exc))

    det_nav_goal.close()
        
if __name__ == "__main__":
    rospy.init_node('navigation_goal_node')
    main()