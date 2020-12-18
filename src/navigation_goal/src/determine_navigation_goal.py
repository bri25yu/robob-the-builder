#!/usr/bin/env python

import rospy
import rospkg
import numpy as np

from nav_msgs.msg import OccupancyGrid
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

		self.buffer = []

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
		for r in range(int((-4.5 - x_origin)//res), int((4.5 - x_origin)//res)):
			for c in range(int((-4.5 - x_origin)//res), int((4.5 - x_origin)//res)):
				if grid[r * cols + c] > 70:
					self.block.x = c * res + x_origin

					# subtract block length so we don't hit the block
					self.block.y = r * res + y_origin

					# top left (neg, neg)
					if (self.block.x <= 0 and self.block.y <= 0):
						self.block.x += 0.4
						self.block.y += 0.4
					# top right (neg, pos)
					elif (self.block.x <= 0 and self.block.y > 0):
						self.block.x += 0.4
						self.block.y -= 0.4
					# bottom right (pos, pos)
					elif (self.block.x > 0 and self.block.y > 0):
						self.block.x -= 0.4
						self.block.y -= 0.4
					# bottom left (pos, neg)
					elif (self.block.x > 0 and self.block.y <= 0):
						self.block.x -= 0.4
						self.block.y += 0.4

					if not self.is_in_buffer(self.block.x, self.block.y):
						found_block = True
						break

			if found_block:
				break

		rospy.loginfo("goal is updated to " + str(self.block.x) + ", " + str(self.block.y))
		if not found_block:
			self.nav_goal_service.shutdown()
			rospy.signal_shutdown("Map is sufficiently explored.")

	def is_in_buffer(self, x, y):
		for block_found in self.buffer:
			x_f, y_f = block_found
			if np.abs(x - x_f) < 0.1 and np.abs(y - y_f) < 0.1:
				return True

		return False

	def get_nav_goal(self, _):
		while (self.block.x == 0 and self.block.y == 0):
			rospy.sleep(1)

		self.buffer.append([self.block.x, self.block.y])
		return self.block

def subscribe_to_map():
	rospy.init_node('det_nav_goal_server')
	rospy.sleep(10)

	nav_goal = Nav_goal()
	rospy.Subscriber("map", OccupancyGrid, nav_goal.det_nav_goal)
	rospy.spin()

if __name__ == '__main__':
	try:
		subscribe_to_map()
	except rospy.ROSInterruptException:
		pass
	
