#!/usr/bin/python

# libraries included in this code
import rospy
import rospy.rostime as rostime
import tf
import numpy as np
import tf.transformations as tfs
import sys
import getopt
import threading
import math
from IPython import embed
import tf2_ros

# message types used in this code
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
# from jaco_teleop.msg import CartVelCmd
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
# from geometry_msgs.msg import Vector3

# pre-defined service messages used in this code 
from kinova_pfields.srv import GeneratePFieldVel, GeneratePFieldVelResponse, GeneratePFieldVelRequest

npa = np.array

# CONSTRUCTOR:  this will get called whenever an instance of this class is created 
# want to put all dirty work of initializations here
class PotentialField(object): 
# constructor
	def __init__(self): 

		#  package up the messy work of creating subscribers; do this overhead in constructor
		self.initializeServices()
		# initializeSubscribers()

		# initialize variables here, as needed 
		# get robot_type from parameter server. 
	 	# we need this for planning, as well as how many velocities are required (based on number of fingers)
		if(rospy.has_param('robot_type')):
			self.robot_type_ = rospy.get_param('robot_type') 
		else:
			rospy.logerr('robot_type rosparam not set!')

		# member variabless
		self.eef_pose_ = Pose()
		self.buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.buffer)
		# self.attractor_linear_vel = Vector3()
		# self.attractor_angular_vel = Vector3() 
		# self.repellor_vel = Vector3() 
		self.attractor_linear_vel = npa([[0]*3], dtype='f')
		self.attractor_angular_vel = npa([[0]*3], dtype='f') # the rotational part of twist
		self.repellor_vel_sum = npa([0]*3, dtype='f') #repelling velocity only to affect the translations.


		# initialize potential field parameters
		self.lin_attractive_scaling_factor_ = 0.1
		self.ang_attractive_scaling_factor_ = 0.6
		self.min_attractive_field_threshold = 0.015
		self.max_attractive_field_radius = 0.1
		self.max_rot_attractive_field_threshold = 0.001
		self.min_rot_attractive_field_threshold = 0.12

		self.goal_reached = False

		# can also do tests/waits to make sure all required services, topics, etc are alive
		self.pfield_vel_ = Twist()
		self.pfield_vel_.linear.x = 0
		self.pfield_vel_.linear.y = 0
		self.pfield_vel_.linear.z = 0
		self.pfield_vel_.angular.x = 0
		self.pfield_vel_.angular.y = 0
		self.pfield_vel_.angular.z = 0

	# Member Functions: 

	# member helper function to help setup services: 
	def initializeServices(self): 
		rospy.loginfo("Initializing Potential Field Services")
		rospy.Service("potentialFieldPlanner", GeneratePFieldVel, self.plannerServiceCallback)

	# member function implementation for a service callback function
	# given a requested goal, the service with generate a velocity for the end effector 
	def plannerServiceCallback(self, req): 
		rospy.loginfo("Inside pfield planner service")

		# 1. update pose
		self.getCurrentPose()
		print self.eef_pose_

		# convert to numpy array for ease of operation 
		self.eef_position = npa([self.eef_pose_.position.x, self.eef_pose_.position.y, self.eef_pose_.position.z])
		self.eef_orientation = npa([self.eef_pose_.orientation.x, self.eef_pose_.orientation.y, self.eef_pose_.orientation.z, self.eef_pose_.orientation.w])
		self.goal_position = npa([req.goal.grasp_pose.position.x, req.goal.grasp_pose.position.y, req.goal.grasp_pose.position.z])
		self.goal_orientation = npa([req.goal.grasp_pose.orientation.x, req.goal.grasp_pose.orientation.y, req.goal.grasp_pose.orientation.z, req.goal.grasp_pose.orientation.w])
		
		# 2. compute velocities
		# Mahdieh To do set charge for wall following and tangential forces
		if (req.goal.charge == 1): 
			self.attractorField()
			rospy.loginfo("goal set, planning attractive field")
		else: 
			rospy.loginfo("No attractive field. Repelling obstacles")
		# self.repellerField()


		# 3. return local velociy and bool true if reached goal, false otherwise
		resp = GeneratePFieldVelResponse()
		resp.robot_vel = self.pfield_vel_
		resp.goal_reached = False
		return resp

	# member function for computing attractive potential field velocity
	# use for moving to goal
	# linear and angular attractive field separately calculated
	def attractorField(self): 
		# Based on algorithm found here: 

		# linear attractive velocity
		# find euclidean distance between goal pose and current pose
		lin_dist = np.linalg.norm(self.goal_position - self.eef_position)
		if (lin_dist < self.min_attractive_field_threshold):
			self.attractor_linear_vel = (np.zeros((1,3)))[0]
		else: 
			if (lin_dist < self.max_attractive_field_radius): 
				self.attractor_linear_vel = (self.goal_position - self.eef_position)
			else: 
				self.attractor_linear_vel = self.lin_attractive_scaling_factor_*(self.goal_position - self.eef_position)/lin_dist


		# angular attractive velocity 
		# find distance between current eef orientation and goal orientation
		diff_quat = tfs.quaternion_multiply(self.goal_orientation, tfs.quaternion_inverse(self.eef_orientation))
		diff_quat = diff_quat / np.linalg.norm(diff_quat)
		self.angle_to_goal = 2*math.acos(diff_quat[3]) # 0 to 2pi. only rotation in one direction.
		if self.angle_to_goal > math.pi: 
			self.angle_to_goal -= 2*math.pi
			self.angle_to_goal = abs(self.angle_to_goal)
			diff_quat = -diff_quat

		self.attractor_angular_vel = (np.zeros((1,3)))[0]
		norm_den = math.sqrt(1 - diff_quat[3]*diff_quat[3])
		if norm_den < self.max_rot_attractive_field_threshold:
			self.attractor_angular_vel[0] = diff_quat[0]
			self.attractor_angular_vel[1] = diff_quat[1]
			self.attractor_angular_vel[2] = diff_quat[2]
		else:
			self.attractor_angular_vel[0] = diff_quat[0]/norm_den
			self.attractor_angular_vel[1] = diff_quat[1]/norm_den
			self.attractor_angular_vel[2] = diff_quat[2]/norm_den
			self.attractor_angular_vel[:] = self.ang_attractive_scaling_factor_*self.attractor_angular_vel[:] #scale the velocity
		if abs(self.angle_to_goal) < self.min_rot_attractive_field_threshold:
			self.attractor_angular_vel = (np.zeros((1,3)))[0]


	# member function for computing repeller potential field velocity
	# use to avoid obstacles
	# def repellerField(self): 

	# member function for tangential potential field
	# use to avoid getting stuck in local minima or for patroling
	# def tangentialField(self): 

	# member function for unform potential field 
	# def uniformField(self):

	# member function for perpendicular potential field 
	# def perpendicularField(self):

	# member function for random noise
    # random walking and to avoid getting stuck in local minima
	# def noiseField(self): 


	def getCurrentPose(self): 
		rospy.loginfo("Getting current pose")
		try: 
			transformObject = self.buffer.lookup_transform(self.robot_type_+'_link_base', self.robot_type_+'_end_effector', rospy.Time(0))
			self.eef_pose_.position = transformObject.transform.translation
			self.eef_pose_.orientation = transformObject.transform.rotation
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rospy.loginfo("Could not find transform from %s to %s", self.robot_type_+"_link_base", self.robot_type_+"_end_effector")
		print self.eef_pose_


def main(): 
	# ROS set-ups:
	rospy.init_node("pfield_planner") # node name
	rospy.loginfo("main: instantiating an object of type PotentialField")

	pf = PotentialField()
	rospy.spin()

if __name__ == '__main__': 
	main()