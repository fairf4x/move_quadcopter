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

    robot_pose_ = node_handle.subscribe("slam_out_pose", 1000, &NavigateQuadrotor::positionUpdate, this);

}

void NavigateQuadrotor::positionUpdate(const geometry_msgs::PoseStamped::ConstPtr& robot_pose)
{
    geometry_msgs::Pose p = robot_pose->pose;
   
    robot_x = p.position.x;
    robot_y = p.position.y;

    robot_angle = atan(2*(p.orientation.x*p.orientation.y+p.orientation.z*p.orientation.w)/(1-2*(p.orientation.x*p.orientation.x+p.orientation.y*p.orientation.y)));

    // angle in radians [ 0, 2*M_PI]
    if (robot_angle < 0)
    {
        robot_angle = 2 * M_PI + robot_angle;
    }

    //ROS_INFO("Update pos X: %f Y: %f Z: %f Angle %f", robot_x, robot_y, p.position.z, robot_angle);
}

float NavigateQuadrotor::getAngleDelta(float targetAngle)
{
    float res = targetAngle - robot_angle;
    
    ROS_INFO_STREAM(    "\nrob angle = " << robot_angle
                    <<  "\ntar angle = " << targetAngle
                    <<  "\ndelta = " << res);
    return res;
}

void NavigateQuadrotor::faceTargetPosition(float tolerance, geometry_msgs::Pose& tp)
{
  // get angle
  float desired_angle = atan2(tp.position.y - robot_y, tp.position.x - robot_x);

  float angle_delta = getAngleDelta(desired_angle);

  // prepare turning message
  geometry_msgs::Twist twist;
  geometry_msgs::Twist empty;
  twist.angular.z = 0.5;
  
  ros::Rate r(10);
  while (angle_delta > tolerance && !as_.isPreemptRequested() && ros::ok() )
  {
      cmd_vel_pub_.publish(twist);

      angle_delta = getAngleDelta(desired_angle);

      ros::spinOnce();
      r.sleep();
  }
  
  cmd_vel_pub_.publish(empty);
      
}

void NavigateQuadrotor::executeNavigate(const hector_move_base::NavigateGoalConstPtr& goal)
{
    geometry_msgs::Twist empty;
    // if we need to stop ongoing navigation request
    if (as_.isPreemptRequested() || !ros::ok())
    {
        cmd_vel_pub_.publish(empty);
        as_.setPreempted();
        return;
    }

    geometry_msgs::Pose targetPos;
    targetPos.position.x = goal->end.x;
    targetPos.position.y = goal->end.y;

    faceTargetPosition(0.4,targetPos);
        
    // get dist

    ROS_INFO("Target position reached.");
    ROS_INFO("Setting action as succesfull.");
    as_.setSucceeded();
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

