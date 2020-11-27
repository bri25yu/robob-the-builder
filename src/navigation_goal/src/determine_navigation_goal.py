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
		self.goal = Point(0, 0, 0)
		self.nav_goal_service = rospy.Service('det_nav_goal', NavGoal, self.get_nav_goal)

	def det_nav_goal(self, world_map):
		# I don't care where I am; I only care about where i need to go
		# could pass it as service into mover node?
		cols = world_map.info.width
		rows = world_map.info.height
		res = world_map.info.resolution
		x_origin = world_map.info.origin.position.x
		y_origin = world_map.info.origin.position.y
		grid = world_map.data

		# map is large, so randomly sample
		# TODO: need to kill node somehow
		found_unknown = False
		for _ in range(NUM_SAMPLES):
			col = np.random.randint(cols)
			row = np.random.randint(rows)
			if grid[row * cols + col] == -1:
				self.goal.x = col * res + x_origin
				self.goal.y = row * res + y_origin
				found_unknown = True
				break
		print("goal is updated to " + str(self.goal.x) + ", " + str(self.goal.y))
		if not found_unknown:
			self.nav_goal_service.shutdown()
			rospy.signal_shutdown("Map is sufficiently explored.")



	def get_nav_goal(self, _):
		print("service is called")
		return self.goal

def subscribe_to_map():
	rospy.init_node('det_nav_goal_server')
	nav_goal = Nav_goal()
	rospy.Subscriber("map", OccupancyGrid, nav_goal.det_nav_goal)
	rospy.spin()

if __name__ == '__main__':
	try:
		subscribe_to_map()
	except rospy.ROSInterruptException:
		pass
