cmake_minimum_required(VERSION 2.8.3)
project(syn_interface_py)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  common_msgs
  sensor_msgs
  image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()
