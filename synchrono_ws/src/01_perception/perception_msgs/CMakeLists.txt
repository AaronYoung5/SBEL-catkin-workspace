cmake_minimum_required(VERSION 2.8.3)
project(perception_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY
  msg
  FILES
  Cone.msg
  ConeImageMap.msg
  ConeXY.msg
  ConeDepthMap.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
