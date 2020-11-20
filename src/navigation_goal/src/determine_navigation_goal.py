#!/usr/bin/env python
import sys

import numpy as np

import rospy
import rospkg

#TODO: remove MapMetaData, encode necessary info in parameter server?
from nav_msgs.msg import OccupancyGrid, MapMetaData

"""
Subscribe to map, map_header
Use array to determine location to go to next
"""
def det_nav_goal(world_map):
	# I don't care where I am; I only care about where i need to go
	# could pass it as service into mover node?
	cols = world_map.info.width
	rows = world_map.info.height

	# map is ridiculously large, so randomly sample?


def subscribe_to_map():
	rospy.Subscriber("map", OccupancyGrid, det_nav_goal)

if __name__ == '__main__':
	rospy.init_node('det_nav_goal')
	try:
		subscribe_to_map()
	except rospy.ROSInterruptException:
		pass
