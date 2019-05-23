cmake_minimum_required(VERSION 2.8.3)
project(path_follower)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
  genmsg
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS rospy std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  nodes/path_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
