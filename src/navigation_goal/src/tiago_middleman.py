#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


publisher = None

def callback(twist):
    rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size=50).publish(twist)

def middleman():
    rospy.Subscriber("cmd_vel", Twist, callback=callback)

    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('tiago_middleman')
    middleman()