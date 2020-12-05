#!/usr/bin/env python
import sys

import numpy as np

import rospy
import rospkg

#TODO: remove MapMetaData, encode necessary info in parameter server?
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Point, Pose
from navigation_goal.srv import NavGoal

"""
Subscribe to map, map_header
Use array to determine location to go to next
"""

# number of points to check for unseen point
NUM_SAMPLES = 1000
class Nav_goal:

	def __init__(self):
		self.holding_block = False
		self.block = Point(0, 0, 0)
		self.nav_goal_service = rospy.Service('det_nav_goal', NavGoal, self.get_nav_goal)

	"""
	new use:
	find an occupied cell inside the range of the walls
	but outside the range of where we'll build
	"""
	def det_nav_goal(self, world_map):
		# I don't care where I am; I only care about where i need to go
		# could pass it as service into mover node?
		cols = world_map.info.width
		rows = world_map.info.height
		res = world_map.info.resolution
		x_origin = world_map.info.origin.position.x
		y_origin = world_map.info.origin.position.y
		grid = world_map.data

		#total size: 20 y, 10 x
		found_block = False
		for r in range(rows // 2):
			for c in range(cols):
				if grid[r * c + c] > 70:
					self.block.x = c * res + x_origin
					self.block.y = r * res + y_origin
					found_block = True
					break

		print("goal is updated to " + str(self.goal.x) + ", " + str(self.goal.y))
		if not found_block:
			self.nav_goal_service.shutdown()
			rospy.signal_shutdown("Map is sufficiently explored.")



	def get_nav_goal(self, _):
		#if self.holding_block:
		#go to next place we wanna put the block
		#else:
		return self.block

def subscribe_to_map():
	rospy.init_node('det_nav_goal_server')
	nav_goal = Nav_goal()
	rospy.Subscriber("map", OccupancyGrid, nav_goal.det_nav_goal)
	rospy.spin()

if __name__ == '__main__':
	"""
	try:
		subscribe_to_map()
	except rospy.ROSInterruptException:
		pass
	"""
