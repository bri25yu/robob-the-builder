#!/usr/bin/env python
import sys

import numpy as np

import rospy
import rospkg

import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from navigation_goal.srv import NavGoal
from geometry_msgs.msg import Twist

NUM_SECONDS_TO_ROTATE = 5
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

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = goal_position
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
    do_a_spin() # allow gmapping to see in all directions
    
    rospy.wait_for_service('det_nav_goal', timeout=120)
    det_nav_goal = rospy.ServiceProxy('det_nav_goal', NavGoal, persistent=True)
    # TODO: do a spin before moving at all

    done = False
    while (not done):
        try:
            goal_position = det_nav_goal()
            move_to_goal(goal_position.position)
            # TODO: pick up block
        except rospy.ServiceException as exc:
            print("Building complete.")
            done = True
            # We're done when det_nav_goal shuts down
            # print("det_nav_goal service did not process request: " + str(exc))

        do_a_spin() # allow gmapping to see in all directions

    det_nav_goal.close()

def do_a_spin():
    velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel')
    vel_msg = Twist()
    
    angular_speed = 2 * np.pi / NUM_SECONDS_TO_ROTATE
    vel_msg.angular.z = angular_speed

    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    relative_angle = 2 * np.pi # 360 degrees
    while (current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed * (t1 - t0)

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)


        
if __name__ == "__main__":
    rospy.init_node('navigation')
    get_nav_goals()