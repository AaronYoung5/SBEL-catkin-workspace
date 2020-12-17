cmake_minimum_required(VERSION 2.8.3)
project(yolo_v3)

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
  nodes/yolo_v3
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
