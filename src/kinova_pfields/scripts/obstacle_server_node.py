#!/usr/bin/env python

# This service provider broadcasts obstacles for every request it gets from the potential field node. 
# Mahdieh TO DO: read from .yaml or from octomap server instead of hardcoding

# rosp libraries 
import rospy

# other ros imports
from kinova_pfields.msg import ObjectDescription
from kinova_pfields.srv import BroadcastObstacles, BroadcastObstaclesResponse, BroadcastObstaclesRequest

class ObstacleBroadcastService():

	def __init__(self):
		# Broadcast service
		rospy.Service("pfields/obstacle_broadcaster", BroadcastObstacles, self.queryKnownObstacles)


	def obstacleBroadcastCB(self, msg):
		self.obstacle_array = msg
		print self.obstacle_array
		print "recieved obstacle array"

	def queryKnownObstacles(self): 
		print "returning obstacles"
		return self.obstacle_array


if __name__ == '__main__':
	rospy.init_node("obstacle_broadcast_server", anonymous=True)
	s = ObstacleBroadcastService()
	# Initialize subscriber
	rospy.Subscriber("pfields/obstacles", ObjectDescription, s.obstacleBroadcastCB)
	rospy.spin()