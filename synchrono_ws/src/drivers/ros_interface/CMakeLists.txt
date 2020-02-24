cmake_minimum_required(VERSION 2.8.3)
project(ros_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  common_msgs
  sensor_msgs
  cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

install(PROGRAMS
  nodes/ros_interface
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
