#ifndef QUADROTOR_EXECUTION_QUADROTOR_HECTOR_NAVIGATION_H
#define QUADROTOR_EXECUTION_QUADROTOR_HECTOR_NAVIGATION_H

class NavigateQuadrotor
{
public:

	NavigateQuadrotor(std::string name, ros::NodeHandle node_handle);
	void executeNavigate(const hector_move_base::NavigateGoalConstPtr & goal);
	void receiveState(const nav_msgs::Odometry::ConstPtr & p);
	actionlib::SimpleActionServer<hector_move_base::NavigateAction> as_; // The action server.
	ros::NodeHandle node_handle;

private:

	float getMaxLocationDiff(const geometry_msgs::PoseStamped & target);
	float getMaxAngleDiff(const float target_angle);

	ros::Publisher cmd_pose_pub_;
	hector_move_base::NavigateFeedback feedback_; // Feedback struct -- returns robot's location.

	ros::Subscriber odom_update_sub_;
	float robot_x;
	float robot_y;
	float robot_altitude;
	float robot_angle;
};

#endif
