#!/usr/bin/env python

import sys
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from kinova_pfields.srv import DisplayObject, DisplayObjectResponse, DisplayObjectRequest
from IPython import embed


class ObjMarkerService():

	def __init__(self):

		# get robot type for frame_id reference
		self.robot_type = rospy.get_param('robot_type')

		# Broadcast service
		rospy.Service("pfields/display_object_marker", DisplayObject, self.displayObjectMarker)
		
		# Initialize topic to be published (to view object in rviz)
		self.marker_pub = rospy.Publisher("object_display_marker", MarkerArray, queue_size = 1)

		# Creat a variable to store the list of objects
		self.object_markers = MarkerArray()

	def displayObjectMarker(self, req):
		marker = Marker()
		marker.header.frame_id = self.robot_type + "_link_base"
		marker.header.stamp = rospy.Time.now()		
		marker.id = req.description.id
		marker.type = req.description.type
		# mnj TO DO: add or remove based on object description? 
		marker.action = marker.ADD
		marker.pose.position = req.description.object_pose.position		
		marker.pose.orientation = req.description.object_pose.orientation
		marker.scale = req.description.scale
		marker.color.a = 0.3
		marker.color.b = 0.0
		if req.description.charge==1: 
			marker.color.r = 0.0 # goal is green 
			marker.color.g = 1.0
			marker.ns = "goal"
		else: 
			marker.color.r = 1.0 # obstacles are red
			marker.color.g = 0.0
			marker.ns = "obstacle"
		self.object_markers.markers.append(marker)
		self.marker_pub.publish(self.object_markers)
		return True

def main():
	s = ObjMarkerService()
	r = rospy.Rate(40)
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node("object_marker_display_server")
	try:
		main()
	except rospy.ROSInterruptException:
		pass
