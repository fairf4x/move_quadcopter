#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <hector_move_base/NavigateAction.h>
#include <actionlib/server/simple_action_server.h>

#include <gazebo_msgs/ModelState.h>

#include "hector_move_base/QuadrotorNavigationServer.h"

NavigateQuadrotor::NavigateQuadrotor(std::string name):
as_(node_handle, name, boost::bind(&NavigateQuadrotor::executeNavigate, this, _1), false)
{
    as_.start();
    ROS_INFO("Navigation service started");
    cmd_vel_pub_ = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    odom_update_sub_ = node_handle.subscribe("/update_drone_odometry",1,&NavigateQuadrotor::ReceiveOdometry, this);
	  quadrotor_ground_truth_sub_ = node_handle.subscribe("/gazebo/model_states", 1, &NavigateQuadrotor::ReceiveGazeboGroundThruth, this);

    robot_x = 0.0;
    robot_y = 1.5;
    robot_angle = 0.0;
}

void NavigateQuadrotor::ReceiveOdometry(const geometry_msgs::Pose::ConstPtr &  p)
{
	// 
    robot_x = p->position.x;
    robot_y = p->position.y;
	
	// Yaw
    robot_angle = atan(2*(p->orientation.x*p->orientation.y+p->orientation.z*p->orientation.w)/(1-2*(p->orientation.x*p->orientation.x+p->orientation.y*p->orientation.y)));
	 
	//robot_x = actual_x;
	//robot_y = actual_y;
	//robot_angle = actual_angle;
    
    if (robot_angle < 0)
    {
        robot_angle = 2 * M_PI + robot_angle;
    }
    
    ROS_INFO("Update Odometry X: %f Y: %f Z: %f Angle %f; Actual: (%f, %f, %f); yaw= %f.", robot_x, robot_y, p->position.z, robot_angle, actual_x, actual_y, actual_z, actual_angle);

}

void NavigateQuadrotor::ReceiveGazeboGroundThruth(const gazebo_msgs::ModelStatesConstPtr& ground_states)
{
	// Search for the index for the quadrotor.
	int index = 0;
	for (std::vector<std::string>::const_iterator ci = ground_states->name.begin(); ci != ground_states->name.end(); ++ci)
	{
		if ("quadrotor" == *ci)
		{
			actual_x = ground_states->pose[index].position.x;
			actual_y = ground_states->pose[index].position.y;
			actual_z = ground_states->pose[index].position.z;
			
			tf::Quaternion q(ground_states->pose[index].orientation.x, ground_states->pose[index].orientation.y, ground_states->pose[index].orientation.z, ground_states->pose[index].orientation.w);
			actual_angle = tf::getYaw(q);
		}
		index++;
	}
}

void NavigateQuadrotor::executeNavigate(const hector_move_base::NavigateGoalConstPtr& goal)
{
    // if we need to stop ongoing navigation request
    if (as_.isPreemptRequested() || !ros::ok())
    {
        as_.setPreempted();
		    return;
    }

	  ROS_INFO_STREAM("New Request: " << goal->end.x << ", "
                                    << goal->end.y << ", "
                                    << goal->height << ", angle "
                                    << goal->end.theta << ", relative="
                                    << (int)goal->relative);

    // get coordinates of the target position
    double x = goal->end.x;
    double y = goal->end.y;

    // compute destination coordinates if the request is relative
    if( goal->relative == true)
    {
        double x_in = goal->end.x+robot_x;
        double y_in = goal->end.y+robot_y;
        std::cout<<"X_in "<< x_in <<", Y_in "<<y_in<<std::endl;
        x = x_in*cos(robot_angle)-y_in*sin(robot_angle)+robot_x-cos(robot_angle)*robot_x+sin(robot_angle)*robot_y;
        y = sin(robot_angle)*x_in+cos(robot_angle)*y_in+robot_y-sin(robot_angle)*robot_x-cos(robot_angle)*robot_y;
    }

    const double drift_x = 1;
    const double drift_angle = 0.3;

    // how much do we have to turn to face the target?
    float desired_angle = atan2(y - robot_y, x - robot_x);
    if (desired_angle < 0)
    {
        desired_angle = 2 * M_PI + desired_angle;
    }

    ROS_INFO_STREAM(  "Goal_x " << x
                  <<  " Goal_y " << y
                  <<  " Desired Angle " << desired_angle );


    float diff_distance = sqrt((robot_x - x) * (robot_x - x) + (robot_y - y) * (robot_y - y));

    float diff_angle = desired_angle - robot_angle;

    // correct computed angle
    if (diff_angle < 0)
    {
        diff_angle = 2 * M_PI + diff_angle;
    }
    
    ROS_INFO_STREAM(  "robot_y "        << robot_y
                  <<  ", robot_x "      << robot_x
                  <<  ", robot_angle "  << robot_angle);
    ROS_INFO_STREAM(  "desired_angle "  << desired_angle
                  <<  ", robot_angle "  << robot_angle
                  <<  ", diff_angle "   << diff_angle);

    // turn until we face the target
    geometry_msgs::Twist twist;
    geometry_msgs::Twist empty;
    
    ROS_INFO("Turning to face the target.");
	  
    if (diff_angle < M_PI)
	  {
      ROS_INFO("diff_angle < M_PI");

      while( diff_angle > drift_angle && !as_.isPreemptRequested() && ros::ok())
      {
        twist.angular.z = drift_angle;
        ros::Duration(1).sleep();
        cmd_vel_pub_.publish(twist);
        ros::Duration(1).sleep();
        cmd_vel_pub_.publish(empty);
        diff_angle = diff_angle - drift_angle;
      }

	
      // terminate if requested
      if (as_.isPreemptRequested() || !ros::ok())
      {
		    cmd_vel_pub_.publish(empty);
        as_.setPreempted();
		    return;
      }

      twist.linear.x = twist.linear.y = twist.linear.z = 0;
      twist.angular.x = twist.angular.y = 0;
      twist.angular.z = diff_angle;


      ros::Duration(1).sleep();
      cmd_vel_pub_.publish(twist);
      ros::Duration(1).sleep();
      cmd_vel_pub_.publish(empty);
	  }
    else
	  {
      ROS_INFO("diff_angle < M_PI");
		
      diff_angle = diff_angle - 2*M_PI;
		
      while( diff_angle < -drift_angle && !as_.isPreemptRequested() && ros::ok())
      {
        twist.angular.z = -drift_angle;
        ros::Duration(1).sleep();
        cmd_vel_pub_.publish(twist);
        ros::Duration(1).sleep();
        cmd_vel_pub_.publish(empty);
        diff_angle = diff_angle + drift_angle;
      }
	
      if (as_.isPreemptRequested() || !ros::ok())
      {
		    cmd_vel_pub_.publish(empty);
        as_.setPreempted();
  		  return;
      }

      twist.linear.x = twist.linear.y = twist.linear.z = 0;
      twist.angular.x = twist.angular.y = 0;
      twist.angular.z = diff_angle;

      ros::Duration(1).sleep();
      cmd_vel_pub_.publish(twist);
      ros::Duration(1).sleep();
      cmd_vel_pub_.publish(empty);
	  }
  
  ROS_INFO("Turning complete.");

  ROS_INFO("Moving the distance towards the target position");

  while( diff_distance > drift_x && !as_.isPreemptRequested() && ros::ok()) {

        ROS_INFO("Diff distance: %f, > drift_x: %f", diff_distance, drift_x);

        twist.linear.x = drift_x;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x=twist.angular.y=twist.angular.z=0;

        ros::Duration(1).sleep();
        cmd_vel_pub_.publish(twist);
        ros::Duration(1).sleep();
        cmd_vel_pub_.publish(empty);

        diff_distance = diff_distance - drift_x;
  }

  ROS_INFO("Target position reached.");
  
  if (as_.isPreemptRequested() || !ros::ok())
    {
		    cmd_vel_pub_.publish(empty);
        as_.setPreempted();
		  return;
    }

  ROS_INFO("Strange code executing..");


    twist.linear.x = diff_distance;

    ros::Duration(1).sleep();
    cmd_vel_pub_.publish(twist);
    ros::Duration(1).sleep();
    cmd_vel_pub_.publish(empty);
    
    //take off  and landing
	
		twist.linear.x = 0;
		twist.linear.y = 0;
		twist.linear.z = goal->height;
		twist.angular.x=twist.angular.y=twist.angular.z=0;

		ros::Duration(2).sleep();
		cmd_vel_pub_.publish(twist);
	
	
	

    ros::Duration(1).sleep();
    cmd_vel_pub_.publish(empty);


    ROS_INFO("setting action as succesfull");

    as_.setSucceeded();

    robot_x = x;
    robot_y = y;
    robot_angle = desired_angle;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "nav_server");
    
    ROS_INFO("hector_move_base main function");

    ros::NodeHandle nh;
    NavigateQuadrotor nq(ros::this_node::getName());

    ros::spin();

    return 0;

}

