#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <hector_move_base/NavigateAction.h>
#include <actionlib/server/simple_action_server.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_navigate_client");

    // TODO
    // advertise service receiving list of [X,Y] coordinated

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<hector_move_base::NavigateAction> ac("nav_server", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Navigation service found.");

    double x [3] = {0.0, 5.0, 0.0};
    double y [3] = {0.0, 5.0, 5.0};

    for( int i = 0; i < 3; i++)
    {
        hector_move_base::NavigateGoal goal;
        goal.end.x = x[i];
        goal.end.y = y[i];

        ac.sendGoalAndWait(goal);
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Navigate action is completed!");
        }
        else
        {
            ROS_ERROR("Navigate action failed!");
        }
    }




    return 0;
}
