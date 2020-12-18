#!/usr/bin/env python

import rospy
from rospy.rostime import Duration
import rospkg
import numpy as np
import actionlib

from std_srvs.srv import Empty
from navigation_goal.srv import NavGoal, GoalDirection

from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Quaternion

from tf.transformations import quaternion_from_euler

NUM_SECONDS_TO_ROTATE = 5

def move_to_goal(goal_position, set_angle=False, returning_to_pickup=False):
    print("move_to_goal is called with" + str(goal_position.x) + " " + str(goal_position.y))
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = goal_position
    if set_angle:
        x, y = goal_position.x, goal_position.y

        if returning_to_pickup:
            goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.pi * 3 / 2))
        # top left (neg, neg)
        elif (x <= 0 and y <= 0):
            goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.pi * 5 / 4))
        # top right (neg, pos)
        elif (x <= 0 and y > 0):
            goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.pi * 3 / 4))
        # bottom right (pos, pos)
        elif (x > 0 and y > 0):
            goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.pi * 1 / 4))
        # bottom left (pos, neg)
        elif (x > 0 and y <= 0):
            goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.pi * 7 / 4))
    else:
        goal.target_pose.pose.orientation.w = 1.0
    
    client.send_goal(goal)
    timeout = Duration()
    timeout.secs = 90
    wait = client.wait_for_result(timeout=timeout)

    client.cancel_all_goals()
    if not wait:
        rospy.logerr("Action server not available!")
        print("returning")
        return None
    else:
        return client.get_result()

def get_nav_goals():
    print("Spinning")
    do_a_spin() # allow gmapping to see in all directions
    print("Finished spinning")
    prepare_robot()
    
    rospy.wait_for_service('det_nav_goal', timeout=120)
    det_nav_goal = rospy.ServiceProxy('det_nav_goal', NavGoal, persistent=True)

    done = False
    while (not done):
        try:
            goal_position = det_nav_goal()
            move_to_goal(goal_position.position, True)
            print("returned from move to goal")
            halt_robot()
            begin_pickup(calc_angle(goal_position.position))

        except rospy.ServiceException as exc:
            print("Building complete.")
            done = True
            # We're done when det_nav_goal shuts down

    det_nav_goal.close()

def calc_angle(goal_position):
    x, y = goal_position.x, goal_position.y

    # top left (neg, neg)
    if (x <= 0 and y <= 0):
        return -1
    # top right (neg, pos)
    elif (x <= 0 and y > 0):
        return 1
    # bottom right (pos, pos)
    elif (x > 0 and y > 0):
        return 1
    # bottom left (pos, neg)
    elif (x > 0 and y <= 0):
        return -1

def rotate_robot(angular_z, time):
    velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=20)
    vel_msg = Twist()
    
    r = rospy.Rate(20) # 10hz

    vel_msg.angular.z = angular_z
    for _ in range(time):
        velocity_publisher.publish(vel_msg)
        r.sleep()

def do_a_spin():
    rotate_robot(2 * np.pi / NUM_SECONDS_TO_ROTATE, 20*NUM_SECONDS_TO_ROTATE)

def halt_robot():
    rotate_robot(0, 25)

def prepare_robot():
    rospy.wait_for_service('/ng_prepare')
    try:
        rospy.loginfo("Moving robot into prepare pos")
        prepare = rospy.ServiceProxy("/ng_prepare", Empty)
        prepare()
    except rospy.ServiceException as e:
        print("Service call to prepare failed: %s" %e)

def begin_pickup(theta):
    rospy.wait_for_service('/ng_init_pickup')
    try:
        rospy.loginfo("Initializing pickup from navigation.py")
        ng_init_pickup = rospy.ServiceProxy("/ng_init_pickup", GoalDirection)
        float_req = Float64()
        float_req.data = theta
        ng_init_pickup(float_req)
    except rospy.ServiceException as e:
        print("Service call to prepare failed: %s" %e)

    rospy.sleep(10)
        
if __name__ == "__main__":
    rospy.init_node('navigation')
    get_nav_goals()