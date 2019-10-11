#!/usr/bin/python

"""
This is an example script for testing the operation of the pfield_planner. 
"""

# rosp libraries 
import rospy

# other ros imports
from kinova_pfields.srv import DisplayObject, DisplayObjectResponse, DisplayObjectRequest
from kinova_pfields.srv import GeneratePFieldVel,  GeneratePFieldVelResponse, GeneratePFieldVelRequest
from kinova_pfields.msg import ObjectDescription

# python libraries
import sys
import argparse
from IPython import embed

class PFieldTests(object): 

	def __init__(self, test_id): 
		# prefix the method_name with 'test_' because method names cannot begin with an integer
		test_name = 'test_'+str(test_id)
		# get test from 'self'. default to a lambda
		test = getattr(self, test_name, lambda: "nothing")

		# Initialize topic to be published (to view object in rviz)
		self.obstacle_pub = rospy.Publisher('pfields/obstacles', ObjectDescription, queue_size = 1)

		# convenience method that blocks until the service is available
		rospy.wait_for_service('potentialFieldPlanner') # pfield planner (required)		
		rospy.wait_for_service('pfields/obstacle_broadcaster') # obstacle broadcaster (required)
		rospy.wait_for_service('pfields/display_object_marker') # visualization (optional)

		try: 
			# create handle for calling the service with ('Planner_Name', Type-of-service)
			self.pfield_plan_request = rospy.ServiceProxy('potentialFieldPlanner', GeneratePFieldVel)
		except rospy.ServiceException, err:
			print "Pfield planner service call failed: %s"%err

		try: 
			self.visualize_object = rospy.ServiceProxy('pfields/display_object_marker', DisplayObject)
		except rospy.ServiceException, err: 
			print "Visualization service call failed: %s"%err	


		# call test as we return it
		return test()


	def test_1(self):
		# Test 1: 
		# send single goal with no obstacles
		rospy.loginfo("in test 1")
		# call set_hard_coded_goal function to get goal
		self.set_hard_coded_goal()
		self.obstacle_pub.publish(self.obstacle)
		self.call_visualization_node(self.goal)		

		#call pfield planner service with desired goal: 
		rospy.loginfo("requesting plan")
		resp1 = self.pfield_plan_request(self.goal)
		rospy.loginfo("recieved pfield plan")
		print resp1.robot_vel
		print resp1.goal_reached

	def test_2(self):
		# Test 2: 
		# send single goal with single hardcoded obstacle
		rospy.loginfo("in test 2")
		# publish hard coded obstacle
		self.set_hard_coded_obstacle()
		self.call_visualization_node(self.obstacle)

		# call set_hard_coded_goal function to get goal
		# self.set_hard_coded_goal()
		# self.call_visualization_node(self.goal)		

		# #call pfield planner service with desired goal: 
		# rospy.loginfo("requesting plan")
		# resp1 = self.pfield_plan_request(self.goal)
		# rospy.loginfo("recieved pfield plan")
		# print resp1.robot_vel
		# print resp1.goal_reached

	def set_hard_coded_goal(self): 
		# goal is hard-coded here
		self.goal = ObjectDescription()
		self.goal.charge = 1 # goal has a charge of 1 for attraction
		# Pose of goal
		self.goal.object_pose.position.x = .5
		self.goal.object_pose.position.y = -.5
		self.goal.object_pose.position.z = .5
		self.goal.object_pose.orientation.x = 0
		self.goal.object_pose.orientation.y = 0
		self.goal.object_pose.orientation.z = 0
		self.goal.object_pose.orientation.w = 0
		# Grasp pose (what direction we want the gripper to approach the goal)
		self.goal.grasp_pose.position.x = .5
		self.goal.grasp_pose.position.y = -.5
		self.goal.grasp_pose.position.z = .5
		self.goal.grasp_pose.orientation.x = 0
		self.goal.grasp_pose.orientation.y = 0
		self.goal.grasp_pose.orientation.z = 0
		self.goal.grasp_pose.orientation.w = 0
		# shape of the goal object CUBE:1, SPHERE:2, CYLINDER:3
		self.goal.type = 2
		# size of object shape: 
		self.goal.scale.x = .2
		self.goal.scale.y = .2
		self.goal.scale.z = .2
		# goal id: 
		self.goal.id = 2


	def set_hard_coded_obstacle(self):
		rospy.loginfo("in set obstacles hardcoded")
		# goal is hard-coded here
		# Note this should ideally be a separate server for publishing obstacles (similar to costmap servre)
		# Note to do: fiugre out how to use with octomap server if possible
		self.obstacle = ObjectDescription()
		self.obstacle.charge = 3 # verticle wall has a "charge" of 3
		# Pose of goal
		self.obstacle.object_pose.position.x = .5
		self.obstacle.object_pose.position.y = -.5
		self.obstacle.object_pose.position.z = .1
		self.obstacle.object_pose.orientation.x = 0
		self.obstacle.object_pose.orientation.y = 0
		self.obstacle.object_pose.orientation.z = 0
		self.obstacle.object_pose.orientation.w = 0
		# shape of the goal object CUBE:1, SPHERE:2, CYLINDER:3
		self.obstacle.type = 1
		# size of object shape: 
		self.obstacle.scale.x = .4
		self.obstacle.scale.y = .4
		self.obstacle.scale.z = .4
		# goal id: 
		self.obstacle.id = 1
		print self.obstacle
		self.obstacle_pub.publish(self.obstacle)

	def call_visualization_node(self, obj): 
		# call visualization service with desired goal or obstalces: 
		rospy.loginfo("visualizing object")
		resp = self.visualize_object(obj)
		rospy.loginfo("object visualization %s for object %d", resp, obj.id)


def main(): 
	rospy.init_node("jaco_pfield_test_node")

	# requires integer argument when running this node
	# integer determines which test to run
	# parse input here
	rospy.myargv(argv=sys.argv)
	parser = argparse.ArgumentParser(description="This is a script to test the pfield_planner node. You can choose which test to run.")
	parser.add_argument('test', metavar='N', type=int, nargs='+', help='test number')
	option = parser.parse_args()
	num_test = len(option.test)
	print "args", option.test[0]

	# now that we have the argument, call the test
	PFieldTests(option.test[0])


	rospy.spin() 

if __name__ == "__main__": 
	main()



