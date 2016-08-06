# move_quadcopter

This experimental package uses actionlib to implement a simple navigation server and client for [hector quadrotor](http://wiki.ros.org/hector_quadrotor).

## Navigation server
Server publishes `/cmd_vel` topic to control quadcopter movement and `/goal_marker` topic to indicate current goal in RVIZ (for debugging purposes).
The goal is set by navigation client.

## Navigation client
Client subscribes to the `/get_goal` topic. New waypoint for a quadcopter to fly to can be specified for example by:

```
rostopic pub -1 /get_goal geometry_msgs/Point "x: -4.0
y: -4.0
z: 1.0"
```

This will trigger `goalCallback` function of the client which sends a `move_quadcopter::NavigateGoal` request to the server together with XYZ coordinates of the waypoint.

### HOWTO

1. install `move_quadcopter` package in your catkin workspace and compile it (`catkin_make --pkg move_quadcopter`) 
2. launch the test launchfile: `roslaunch move_quadcopter indoor_test_hector_quadcopter.launch`
3. take off (either by publishing to messages to `/cmd_vel` topic or by using joystick - see the comment in `indoor_test_hector_quadcopter.launch`)
4. publish `geometry_msgs/Point` on `/get_goal` topic

The quadcopter should start to move towards the goal marker (visible as small green sphere in RVIZ).

### Remarks
- The `Z` coordinate is not used when navigating
- navigation action is considered successfull by the server if the distance from the goal is less than 0.1 (see `NavigationServer::executeNavigate`)
