#ifndef QUADROTOR_EXECUTURE_QUADROTOR_NAVIGATION_SERVER_H
#define QUADROTOR_EXECUTURE_QUADROTOR_NAVIGATION_SERVER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <hector_move_base/NavigateAction.h>
#include <actionlib/server/simple_action_server.h>
#include <gazebo_msgs/ModelStates.h>

class NavigateQuadrotor
{
public:

	NavigateQuadrotor(std::string name);
	void executeNavigate(const hector_move_base::NavigateGoalConstPtr& goal);
	void ReceiveOdometry(const geometry_msgs::Pose::ConstPtr & p);
	void ReceiveGazeboGroundThruth(const gazebo_msgs::ModelStatesConstPtr& ground_states);

private:

	ros::NodeHandle node_handle;
	ros::Publisher cmd_vel_pub_;
	ros::Subscriber odom_update_sub_;
	ros::Subscriber quadrotor_ground_truth_sub_;
	
	hector_move_base::NavigateFeedback feedback_; // Feedback struct -- returns robot's location.
	actionlib::SimpleActionServer<hector_move_base::NavigateAction> as_; // The action server.
	
	float robot_x;
	float robot_y;
	float robot_angle;

	float actual_x, actual_y, actual_z, actual_angle;
};

#endif
