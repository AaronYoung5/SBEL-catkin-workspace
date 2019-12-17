cmake_minimum_required(VERSION 2.8.3)
project(camera_visualizer)

find_package(catkin REQUIRED COMPONENTS
  rospy
  common_msgs
  sensor_msgs
  cv_bridge
)

find_package( OpenCV 3 REQUIRED )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

install(PROGRAMS
  nodes/camera_visualizer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
