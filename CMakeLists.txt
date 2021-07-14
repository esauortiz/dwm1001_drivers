cmake_minimum_required(VERSION 2.8.3)

project(dwm1001_drivers)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  uwb_msgs
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  uwb_msgs
 )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs uwb_msgs
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
