#!/usr/bin/python

"""
Author: Deepak E. Gopinath
Program: jaco_pfields_node.py
Date Created: September 18 2016 adapted by Mahdieh Nejati on April 30, 2018

This node represents the velocity field for a particular goal and obstacle configuration. ROS services
are used for querying the j2s7s300_end_effector effector velocity given a robot position.
"""

import rospy
import rospy.rostime as rostime

import tf
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from jaco_pfields.srv import GoalsObs, GoalsObsResponse, GoalsObsRequest
from jaco_pfields.srv import QueryVel, QueryVelResponse, QueryVelRequest

import numpy as np
import tf.transformations as tfs
import sys
import getopt
import threading
import math
from IPython import embed
import tf2_ros

npa = np.array

class PotentialFields(object):

	def __init__(self, num_objs):
		self.num_objs = num_objs
		self.num_goals = 1
		self.num_obstacles = self.num_objs - self.num_goals # n-1
		self.num_dof = 9
		self.cylinder_rad = 0.3
		self.cylinder_h = 0.4

		self.rate = rospy.Rate(50)
		self.buffer = tf2_ros.Buffer()
		# self.listener = tf.TransformListener(True, rospy.Duration(30))
		self.listener = tf2_ros.TransformListener(self.buffer)
		self.running = True
		self.runningCV = threading.Condition()

		#initialize the goal pos and quat arrays and the obstacle pos and quat arrays
		self.goal_positions = npa([0]*3, dtype ='f')
		self.goal_quats = npa([0]*4, dtype = 'f')
		self.goal_orient = 1
		self.dist_to_goal = 0.0
		self.theta_to_goals = 0.0

		self.obs_positions = npa([[0]*3]*self.num_obstacles, dtype= 'f')
		self.obs_quats = npa([[0]*4]*self.num_obstacles, dtype ='f')
		self.dist_to_obs = [0]*self.num_obstacles
		self.obs_threshold = 0.12 # to be tuned properly

		self.eef_position = npa([0]*3, dtype ='f')
		self.eef_quat = npa([0]*4, dtype='f')

		self.pfieldvel_array = npa([[0]*3], dtype='f')
		self.pfieldrot_array = npa([[0]*3], dtype='f') # the rotational part of twist
		self.repel_sum = npa([0]*3, dtype='f') #repelling velocity only to affect the translations.

		self.pfield_vel = Float32MultiArray()
		_dim = [MultiArrayDimension()]
		_dim[0].label = 'cartesian_velocity'
		_dim[0].size = 9 # x y z roll pitch yaw x3 fingers
		_dim[0].stride = 9
		self.pfield_vel.layout.dim = _dim
		self.pfield_vel.data = [0]*self.num_dof #initialize the velocity to zero values.
		self.pfield_vel.data[6] = 0
		self.pfield_vel.data[7] = 0
		self.pfield_vel.data[8] = 0

        # Flags
		self.stage = 3
		self.isLtoR = False
		self.isRtoL = False
		self.istransdone = False
		self.isrotdone = False
		self.iswithintable = False
		self.iswithinobs = False
		self.iswithinbase = False

		self.model_source_frameid = "j2s7s300_link_base"

		rospy.Service(rospy.get_namespace() + "pfields/setgoalsandobs", GoalsObs, self.setGoalsAndObs)
		rospy.Service(rospy.get_namespace() + "pfields/queryrobotpolicy", QueryVel, self.queryRobotPolicy)


        #root policy engine
	def queryRobotPolicy(self, queryvel):

		# print "Goal Position", self.goal_positions
		self.update_current_position(queryvel.robot_pose.position, queryvel.robot_pose.orientation)
		self.reset_flags()
		self.compute_stage() #updating the stage happens from compute_stage
		self.compute_current_robot_velocity()

		#at this stage you have the velocity ready
		#package this velocity into the service response

		if self.isintrans:
			# print "In trnas 1"
			self.pfield_vel.data[:3] = list(self.pfieldvel_array[:]) #pick the first goals total vel and assign it to pfieldvel
		else:
			# print "In trans 2"
			self.pfield_vel.data[:3] = [0]*3

		if self.isinrot:
			self.pfield_vel.data[3:6] = list(self.pfieldrot_array[:])
			if self.iswithintable: #if within table or obstacle or both, stop rotating and only translate
				self.pfield_vel.data[:3] = list(self.pfieldvel_array[:])
				self.pfield_vel.data[3:6] = [0]*3
				self.iswithintable = False
			if self.iswithinobs:
				self.pfield_vel.data[:3] = list(self.pfieldvel_array[:])
				self.pfield_vel.data[3:6] = [0]*3
				self.iswithinobs = False
		else:
			self.pfield_vel.data[3:6] = [0]*3

		# print self.pfield_vel.data
		if (npa(self.pfield_vel.data[:6]) == 0.0).nonzero()[0].size == 6:
			self.istransdone = True
			self.isrotdone = True
			print "Reached goal in namespace", rospy.get_namespace()

		#prepare the final package to be returned
		final_vel = QueryVelResponse()
		d = [MultiArrayDimension()]
		d[0].label = 'cartesian_velocity'
		d[0].size = 9
		d[0].stride = 9
		final_vel.robot_vel.layout.dim = d
		final_vel.robot_vel.data = [0]*self.num_dof
		final_vel.robot_vel.data[:] = self.pfield_vel.data[:]
		# print "Final Velocity", final_vel

		return final_vel



#component functions for policy engine.

	def update_current_position(self, position, orientation):
		self.eef_position = npa([position.x, position.y, position.z])
		self.eef_quat = npa([orientation.x, orientation.y, orientation.z, orientation.w])

	def reset_flags(self):
		self.istransdone = False
		self.isrotdone = False

	def compute_stage(self): #to check what stage the exeution should be in based on where the robot is at the moment and the goal
		self.isLtoR = False
		self.isRtoL = False
		# if self.dist_of_traj_origin() < 0.35:
		# 	if self.eef_position[0] > 0 and self.goal_positions[0] < 0: #check if ee and goals are on either side of the base
		# 		self.isLtoR = True
		# 	else:
		# 		self.isLtoR = False
		#
		# 	if self.eef_position[0] < 0 and self.goal_positions[0] > 0:
		# 		self.isRtoL = True
		# 	else:
		# 		self.isRtoL = False

		if (not self.isLtoR) and (not self.isRtoL):
			self.stage = 3 #on the same side, both rot and trans gonna happen simultaneously or can be on either side but outside the restricted cylinder zone
			self.update_stage()
		else:
			self.stage = 3 #on either side and line passes through the restricted cylinder zone.
			self.update_stage()

	def update_stage(self):
		if self.stage == 1:
			self.isinrot = True
			self.isintrans = False
			self.select_orientation(2) #siwtch to over the top. This step will be based on whether there is a LtoR or RtoL movement
		elif self.stage == 2:
			self.select_orientation(2)
			self.isinrot = True
			self.isintrans = True
		elif self.stage == 3:
			# TO DO ::: select_orientation should query another node or param server
			self.select_orientation(self.goal_orient) #whatever the destination orientation is
			self.isinrot = True
			self.isintrans = True

#helper functions
	def dist_of_traj_origin(self):
			v = self.eef_position[:2] #ee x,y
			w = self.goal_positions[:2] #goal x,y
			p =  npa([0]*2, dtype='f')  #define origin
			l2 = float(np.linalg.norm(v - w))
			if l2 == 0.0: # if v and w are exactly the same. might have to relax this a little bit for nearby points.
				return np.linalg.norm(v, p)
			else:
				t = max(0, min(1, float(np.dot(p - v, w- v))/l2))
				projection =  v + t*(w -v)
				return np.linalg.norm(p - projection)

#velocity computations
	def compute_current_robot_velocity(self):
		#attractor stuff
		self.update_attractor_vels()
		self.update_attractor_rotvels()

		# #repeller stuff
		self.update_repeller_vels()
		self.update_base_collision() # trans vel for collision with base.
		self.update_table_collision()

		# #add attractor and repeller trans
		self.update_total_vels()


	def update_attractor_vels(self):
		self.dist_to_goal = np.linalg.norm(self.goal_positions - self.eef_position)
		self.pfieldvel_array = (np.zeros((1,3)))[0] #might be unnecessary
		if np.linalg.norm(self.goal_positions - self.eef_position) < 0.10:
			self.pfieldvel_array = (self.goal_positions - self.eef_position)
		else:
			self.pfieldvel_array = 0.1*(self.goal_positions - self.eef_position)/(np.linalg.norm(self.goal_positions - self.eef_position))

		# self.pfieldvel_array = 1.5*(self.goal_positions - self.eef_position) #this is the attractor translational velocity
		if self.stage == 2:
			if np.linalg.norm(self.goal_positions - self.eef_position) < 0.1:
				self.stage = 3
				print "Trans got into stage 3"
				self.update_stage()
				self.pfieldvel_array = (np.zeros((1,3)))[0]

		if self.stage == 3:
			if np.linalg.norm(self.goal_positions - self.eef_position) < 0.015: #stringent condition
				self.pfieldvel_array = (np.zeros((1,3)))[0]
			else:
				if np.linalg.norm(self.goal_positions - self.eef_position) < 0.10:
					# print "Non capped ", rospy.get_namespace()
					self.pfieldvel_array = (self.goal_positions - self.eef_position)
				else:
					# print "Capped ", rospy.get_namespace()
					self.pfieldvel_array = 0.1*(self.goal_positions - self.eef_position)/(np.linalg.norm(self.goal_positions - self.eef_position))

	def update_attractor_rotvels(self):
		diff_quat = tfs.quaternion_multiply(self.goal_quats, tfs.quaternion_inverse(self.eef_quat))
		diff_quat = diff_quat/np.linalg.norm(diff_quat)
		self.theta_to_goals = 2*math.acos(diff_quat[3]) #0 to 2pi. only rotation in one direction.
		if self.theta_to_goals > math.pi:
			self.theta_to_goals -= 2*math.pi
			self.theta_to_goals = abs(self.theta_to_goals)
			diff_quat = -diff_quat

		self.pfieldrot_array = (np.zeros((1,3)))[0]
		norm_den = math.sqrt(1 - diff_quat[3]*diff_quat[3])
		if norm_den < 0.001:
			self.pfieldrot_array[0] = diff_quat[0]
			self.pfieldrot_array[1] = diff_quat[1]
			self.pfieldrot_array[2] = diff_quat[2]
		else:
			self.pfieldrot_array[0] = diff_quat[0]/norm_den
			self.pfieldrot_array[1] = diff_quat[1]/norm_den
			self.pfieldrot_array[2] = diff_quat[2]/norm_den
			self.pfieldrot_array[:] = 0.375*self.pfieldrot_array[:] #sclae the velocity

		# if self.stage == 1:
		# 	if abs(self.theta_to_goals) < 0.1:
		# 		self.stage = 2
		# 		self.update_stage()
		# 		print "Got into stage 2"
		# 		self.pfieldrot_array = (np.zeros((1,3)))[0]
		# 	else:
		# 		if abs(self.theta_to_goals) < 0.12:
		# 			self.pfieldrot_array = (np.zeros((1,3)))[0]
		if abs(self.theta_to_goals) < 0.12:
			self.pfieldrot_array = (np.zeros((1,3)))[0]

	def update_repeller_vels(self):
		self.repel_sum = npa([0]*3, dtype='f')
		for i in range(0, self.num_obstacles):
			self.dist_to_obs[i] = np.linalg.norm(self.eef_position-self.obs_positions[i])
			if self.dist_to_obs[i] < self.obs_threshold:
				self.iswithinobs = True
				self.repel_sum = self.repel_sum + (self.eef_position - self.obs_positions[i])/(14*(self.dist_to_obs[i])**2)
		if np.linalg.norm(self.repel_sum) > 0.15:
			self.repel_sum = 0.15*self.repel_sum/np.linalg.norm(self.repel_sum)
			# print "Capped repel vel ", self.repel_sum, rospy.get_namespace()

	def update_base_collision(self):
		tfpose = self.buffer.lookup_transform( self.model_source_frameid, 'j2s7s300_link_7',rospy.Time(0), rospy.Duration(10))
		l5 = npa([tfpose.transform.translation.x, tfpose.transform.translation.y, tfpose.transform.translation.z], dtype = 'f')
		# l5 = npa(self.listener.lookupTransform(self.model_source_frameid, "j2s7s300_link_7", rostime.Time(0))[0])
		# if np.linalg.norm(self.eef_position[:2]) < self.cylinder_rad and self.eef_position[2] < self.cylinder_h:
		# 	stretch_matrix = np.array([[0, 0],[0, 3]], np.float) #push the velocity vector further along y so that the robot moves away from the base more aggressively.
		# 	stretch_vec = np.dot(stretch_matrix, self.eef_position[:2])
		# 	self.repel_sum[:2] = self.repel_sum[:2] + (stretch_vec)/(3*np.linalg.norm(stretch_vec)**2)
		# 	if self.eef_position[1] > 0:
		# 		print "Wrong side"
		# 		self.repel_sum[1] = -math.fabs(self.repel_sum[1]) # reverse direction of y direction vel
		# 	self.iswithinbase = True
		if np.linalg.norm(l5[:2]) < self.cylinder_rad and l5[2] < self.cylinder_h:
			stretch_matrix = np.array([[0, 0],[0, 0.1]], np.float) #push the velocity vector further along -y so that the robot moves away from the base more aggressively.
			stretch_vec = np.dot(stretch_matrix, l5[:2])
			# print "stretch vec",(stretch_vec)/(12*np.linalg.norm(stretch_vec))
			self.repel_sum[:2] = self.repel_sum[:2] + (stretch_vec)/(12*np.linalg.norm(stretch_vec))
			# print "rEpel vel", self.repel_sum
			if l5[1] > 0:
				print "Wrong side"
				self.repel_sum[1] = -math.fabs(self.repel_sum[1]) # reverse direction of y direction vel
				# print "rEpel vel", self.repel_sum
			self.iswithinbase = True

	def update_table_collision(self):
		tfpose_f1 = self.buffer.lookup_transform(self.model_source_frameid, 'j2s7s300_link_finger_tip_1', rospy.Time(0), rospy.Duration(10))
		f1 = npa([tfpose_f1.transform.translation.x, tfpose_f1.transform.translation.y, tfpose_f1.transform.translation.z], dtype = 'f')
		tfpose_f2 = self.buffer.lookup_transform(self.model_source_frameid, 'j2s7s300_link_finger_tip_2', rospy.Time(0), rospy.Duration(10))
		f2 = npa([tfpose_f2.transform.translation.x, tfpose_f2.transform.translation.y, tfpose_f2.transform.translation.z], dtype = 'f')
		tfpose_f3 = self.buffer.lookup_transform(self.model_source_frameid, 'j2s7s300_link_finger_tip_3', rospy.Time(0), rospy.Duration(10))
		f3 = npa([tfpose_f3.transform.translation.x, tfpose_f3.transform.translation.y, tfpose_f3.transform.translation.z], dtype = 'f')

		# f1 = npa(self.listener.lookupTransform(self.model_source_frameid,"j2s7s300_link_finger_tip_1", rostime.Time(0))[0]) #get the translation poisition of one finger tip
		# f2 = npa(self.listener.lookupTransform(self.model_source_frameid,"j2s7s300_link_finger_tip_2", rostime.Time(0))[0])
		# f3 = npa(self.listener.lookupTransform(self.model_source_frameid,"j2s7s300_link_finger_tip_3", rostime.Time(0))[0])

		if f1[2] < 0.04 or f2[2] < 0.04 or f3[2] < 0.04: # if the finger tips are close to the table. Just apply upward velocity. No decay or anything right now
			self.repel_sum[2] = 0.5
			self.iswithintable =  True

	def update_total_vels(self):
		if not self.iswithinbase:
			self.pfieldvel_array[:] = self.pfieldvel_array[:] + self.repel_sum
		else:
			self.pfieldvel_array[:] = self.pfieldvel_array[:] + self.repel_sum # same result on either if block
			self.iswithinbase = False

#setting goals and obstacles
	def select_orientation(self, goal_orient):
		# print "Goal orient", goal_orient
		if goal_orient == 1: #flat
			self.goal_quats[0] = 0.524
			self.goal_quats[1] = -0.490
			self.goal_quats[2] =  0.548
			self.goal_quats[3] =  0.430
			self.cylinder_rad = 0.25
		elif goal_orient == 2: #over-the-top
			self.goal_quats[0] = 0.5841
			self.goal_quats[1] = 5.801
			self.goal_quats[2] = 3.789
			self.goal_quats[3] = 3.742
			self.cylinder_rad = 0.25
		elif goal_orient == 3: #doorknob
			self.goal_quats[0] = 0.521
 			self.goal_quats[1] = 7.665
 			self.goal_quats[2] = 4.625
 			self.goal_quats[3] = 5.146
 			self.cylinder_rad = 0.25
 		else:
 			self.goal_quats[0] = 0.724
 			self.goal_quats[1] = -0.683
 			self.goal_quats[2] = 0.016
 			self.goal_quats[3] = 0.093
 			self.cylinder_rad = 0.25

	def setGoalsAndObs(self, goalsobs):
		print "In set goals and obstacles service in namespace", rospy.get_namespace()
		print "Goal origin", goalsobs.objects
		print "Goal Index ", goalsobs.goal_index
		#Set goal position
		self.goal_positions[0] = goalsobs.objects[goalsobs.goal_index].origin.x
		self.goal_positions[1] = goalsobs.objects[goalsobs.goal_index].origin.y
		self.goal_positions[2] = goalsobs.objects[goalsobs.goal_index].origin.z
		self.goal_orient = goalsobs.objects[goalsobs.goal_index].orientation

		# embed()
		self.select_orientation(goalsobs.objects[goalsobs.goal_index].orientation)

		obs_list = [x for x in range(0, self.num_objs) if x != goalsobs.goal_index]
		ind = 0
		for i in obs_list:
			self.obs_positions[ind][0] = goalsobs.objects[i].origin.x
			self.obs_positions[ind][1] = goalsobs.objects[i].origin.y
			self.obs_positions[ind][2] = goalsobs.objects[i].origin.z
			# obstacle orientation maybe set or need not be set. p fields don't care about it yet
			ind = ind + 1


		print "Goal Position", self.goal_positions, self.goal_quats
		print "Obstacles ", self.obs_positions
		status = GoalsObsResponse()
		status = True
		return status


def main():
	rospy.myargv(argv=sys.argv)
	(options, args) = getopt.getopt(sys.argv[1:], 'n:', ['num_objects='])

	rospy.init_node("jaco_pfields_node")
	rospy.loginfo("In jaco_pfields_node node")
	print "Namespace of this node is ", rospy.get_namespace()


	num_objs = 1
	for o,a in options:
		if o in ('-n','--num_objects'):
			num_objs = int(a)

	pf = PotentialFields(num_objs)
	rospy.spin()



if __name__ == '__main__':
	main()
