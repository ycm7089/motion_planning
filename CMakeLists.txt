cmake_minimum_required(VERSION 3.0.2)
project(motion_planning)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  tf
  tf_conversions
  message_generation
  visualization_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)
 
catkin_package(
 LIBRARIES motion_planning
 CATKIN_DEPENDS rospy std_msgs tf nav_msgs message_runtime visualization_msgs
 DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)