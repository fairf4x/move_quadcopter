# hector_move_base

This experimental package uses actionlib to implement a simple navigation server and client for hector quadrotor.

No mapping or odometry is used. The server accepts 2D coordinates as navigation goal and
controls the quadrotor by publishing on the ```/cmd_vel``` topic.

The client is for testing purposes only. It sends three navigation goals and exits.
