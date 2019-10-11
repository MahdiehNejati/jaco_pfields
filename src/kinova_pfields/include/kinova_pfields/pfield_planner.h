
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
#ifndef Potential_Field_H_
#define Potential_Field_H_

#include <ros/ros.h> //ALWAYS need to include this for ros nodes

// libraries included in this code
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <tf/transform_listener.h>

// message types used in this code
#include <std_msgs/Bool.h> 
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>

// pre-defined service messages used in this code 
#include <kinova_pfields/GeneratePFieldVel.h> 
#include <kinova_pfields/ObjectDescription.h> 


// define class, including constructor, member variables, and meber functions
class PotentialField
{
public: 
	PotentialField(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired

private: 
    // put private member data here;  "private" data will only be available to member functions of this class;
	ros::NodeHandle nh_; // we will need this to pass between "main" and constructor

	// some objects to support subscriber, service, and publisher
    //these will be set up within the class constructor, hiding these ugly details
    //ros::Subscriber subscriber_; 
    //ros::Publisher  publisher_;
    ros::ServiceServer potential_field_service_;
    ros::ServiceClient client; 

    // member variable --- better than using globals; convenient way to pass data from a subscriber to other member functions
	// member variables will retain their values even as callbacks come and go
	geometry_msgs::Twist pfield_vel_;
	geometry_msgs::Vector3 attractor_linear_vel; 
	geometry_msgs::Vector3 attractor_angular_vel; 
	geometry_msgs::Vector3 repellor_vel; 
	std::string robot_type_;
	int num_dof_; 
	sensor_msgs::JointState joint_state_; 
	tf::TransformListener tf_listener; // transform listener object
    tf::StampedTransform transform; // object to store the resulting transform 
    geometry_msgs::TransformStamped eef_pose_;
    float attractive_scaling_factor_;
    float dist_to_goal_; 
    kinova_pfields::ObjectDescription req_goal_; 
    float min_field_threshold; 
    float max_field_radius; 

	// member methods: 
	// we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
	// void initializeSubscribers(); 
	//void initializePublishers(); 
	void initializeServices(); 

	// subscriber callback
	void jointStateCallback(const sensor_msgs::JointState& msg);

	// service callback 
	bool plannerServiceCallback(kinova_pfields::GeneratePFieldVel::Request& req, kinova_pfields::GeneratePFieldVel::Response& res);

	// these functions do all the work. 
	void repellerField();
	void attractorField();
	void tangentialField();
	void uniformField();
	void perpendicularField();
	void noiseField();
	void getCurrentPose(); 
	double computeEuclideanDistance(geometry_msgs::TransformStamped current_pose, kinova_pfields::ObjectDescription goal_pose);
	double computeAngularDistance(geometry_msgs::TransformStamped current_pose, kinova_pfields::ObjectDescription goal_pose);
}; // note: a class definition requires a semicolon at the end of the definition
 
// close header-include ... ALWAYS need one of these to match #ifndef
#endif 