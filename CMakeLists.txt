cmake_minimum_required(VERSION 2.8.3)
project(hector_move_base)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
)

add_action_files(DIRECTORY action FILES
Navigate.action
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(quadrotor_navigation_server src/QuadrotorNavigationServer.cpp)
target_link_libraries(quadrotor_navigation_server ${catkin_LIBRARIES})
add_dependencies(quadrotor_navigation_server ${PROJECT_NAME}_generate_messages_cpp)

add_executable(quadrotor_navigation_client src/QuadrotorNavigationClient.cpp)
target_link_libraries(quadrotor_navigation_client ${catkin_LIBRARIES})
add_dependencies(quadrotor_navigation_client ${PROJECT_NAME}_generate_messages_cpp)
