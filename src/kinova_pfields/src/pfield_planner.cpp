#include "kinova_pfields/pfield_planner.h"

// CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
PotentialField::PotentialField(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor

    // initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    // initializePublishers();
    initializeServices();
    
    // initialize variables here, as needed 
    // get robot_type from parameter server. 
    // we need this for planning, as well as how many velocities are required (based on number of fingers)
    if(nh_.hasParam("robot_type")){
        nh_.getParam("robot_type", robot_type_); 
    }
    else
    {
        ROS_ERROR("robot_type rosparam not set!");
    }

    // Potential field parameters
    attractive_scaling_factor_ = 0.1; 
    min_field_threshold = 0.015; 
    max_field_radius = 0.1; 

    // initializing pfield velocity response
    pfield_vel_.linear.x = 0;
    pfield_vel_.linear.y = 0;
    pfield_vel_.linear.z = 0;
    pfield_vel_.angular.x = 0;
    pfield_vel_.angular.y = 0;
    pfield_vel_.angular.z = 0;    

    // can also do tests/waits to make sure all required services, topics, etc are alive
    
}

// member helper function to help setup services: 
// note odd syntax: &PotentialField::plannerServiceCallback is a pointer to a member function of PotentialField class
// "this" keyword is required, to refer to the current instance of PotentialField
void PotentialField::initializeServices()
{
    ROS_INFO("Initializing Potential Field Services"); 
    potential_field_service_ = nh_.advertiseService("potentialFieldPlanner", &PotentialField::plannerServiceCallback, this); 
}

//member helper function to set up subscribers;
// void PotentialField::initializeSubscribers()
// {
// }

//member helper function to set up publishers;
// note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
// void PotentialField::initializePublishers()
// {
// }

void PotentialField::jointStateCallback(const sensor_msgs::JointState& msg)
{
    joint_state_ = msg; 
    ROS_ERROR_STREAM("recieved joint state" << joint_state_); 
}

//member function implementation for a service callback function
// given a requested goal, the service with generate a velocity for the end effector 
bool PotentialField::plannerServiceCallback(kinova_pfields::GeneratePFieldVel::Request &req, kinova_pfields::GeneratePFieldVel::Response &res)
{
    ROS_INFO("Inside service");

    // 1. update pose
    getCurrentPose(); 

    // 2. compute velocities
    req_goal_ = req.goal; 

    // Mahdieh To do set charge for wall following and tangential forces
    if (req_goal_.charge==1)
    {
        attractorField(); 
        ROS_INFO("Goal set, planning attractive field");
    }
    else ROS_INFO("No attractive field. Repelling obstacles");

    
    return true;
}


// member function for computing attractive potential field velocity
// use for moving to goal
// linear and angular attractive field separately calculated
void PotentialField::attractorField()
{
    // Based on algorithm found here: 

    // linear attractive velocity
    // find euclidean distance between goal pose and current pose
    double lin_dist = computeEuclideanDistance(eef_pose_, req_goal_); 
    if (lin_dist < min_field_threshold)
    {
        attractor_linear_vel.x = 0; 
        attractor_linear_vel.y = 0; 
        attractor_linear_vel.z = 0; 
    }
    else 
    {
        if (lin_dist < max_field_radius)
        {
            attractor_linear_vel.x = abs(eef_pose_.transform.translation.x - req_goal_.grasp_pose.position.x); 
            attractor_linear_vel.y = abs(eef_pose_.transform.translation.y - req_goal_.grasp_pose.position.y); 
            attractor_linear_vel.z = abs(eef_pose_.transform.translation.z - req_goal_.grasp_pose.position.z); 
        }
        else
        {
            attractor_linear_vel.x = attractive_scaling_factor_*abs(eef_pose_.transform.translation.x - req_goal_.grasp_pose.position.x)/lin_dist; 
            attractor_linear_vel.y = attractive_scaling_factor_*abs(eef_pose_.transform.translation.y - req_goal_.grasp_pose.position.y)/lin_dist; 
            attractor_linear_vel.z = attractive_scaling_factor_*abs(eef_pose_.transform.translation.z - req_goal_.grasp_pose.position.z)/lin_dist; 
        }
    }

    // angular attractive velocity
    // find euclidean distance between goal pose and current pose
    double ang_dist = computeEuclideanDistance(eef_pose_, req_goal_); 


}  


// member function for computing repeller potential field velocity
// use to avoid obstacles
void PotentialField::repellerField()
{
    // Based on algorithm found here: 


}


// member function for tangential potential field
// use to avoid getting stuck in local minima or for patroling
void PotentialField::tangentialField()
{

}

// member function for unform potential field 
// use for
void PotentialField::uniformField()
{

}

// member function 
void PotentialField::perpendicularField()
{

}

// member function for random noise
// random walking and to avoid getting stuck in local minima
void PotentialField::noiseField()
{

}

// member function for computing euclidean distance between current eef pose and goal pose
double PotentialField::computeEuclideanDistance(geometry_msgs::TransformStamped current_pose, kinova_pfields::ObjectDescription goal_pose)
{   
    double dist; 
    dist = sqrt(pow(current_pose.transform.translation.x - goal_pose.grasp_pose.position.x, 2) + 
           pow(current_pose.transform.translation.y - goal_pose.grasp_pose.position.y, 2) + 
           pow(current_pose.transform.translation.z - goal_pose.grasp_pose.position.z, 2));    
    return dist; 
}

// member function for computing quaternion distance between current eef orientation and goal orientation
double PotentialField::computeAngularDistance(geometry_msgs::TransformStamped current_pose, kinova_pfields::ObjectDescription goal_pose)
{   
    double dist; 
    // tf::Quaternion::inverse(current_pose.transform.rotation) 
    ROS_ERROR_STREAM("original quat: " << current_pose.transform.rotation); 
    ROS_ERROR_STREAM("inverse quat: " << tf::Quaternion::inverse(current_pose.transform.rotation) ); 
    return dist; 
}

// member function for geting current robot pose to calculate velocites
void PotentialField::getCurrentPose()
{   
    ROS_INFO("Getting current pose");
    
    try{
        // we want the transform from robot_type_end_effector to the world frame, at the latest available time, stored in transform
        ros::Time now = ros::Time::now(); 
        tf_listener.waitForTransform("world", robot_type_+"_end_effector", now, ros::Duration(3.0)); 
        tf_listener.lookupTransform("world", robot_type_+"_end_effector", ros::Time(0), transform); 
        transformStampedTFToMsg(transform, eef_pose_); 
        // ROS_ERROR_STREAM("robot pose with tf: " << eef_pose_);
    }
    catch (tf::TransformException &ex){
        ROS_ERROR("%s", ex.what()); 
    }
}

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "pfield_planner"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type PotentialField");
    PotentialField potentialField(&nh);  //instantiate an PotentialField object and pass in pointer to nodehandle for constructor to use

    ros::AsyncSpinner spinner(1); 

    while(ros::ok()){
        // going into spin, let the callbacks do all the work
        spinner.start(); 
    }
    
    return 0; 
} 