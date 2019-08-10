cmake_minimum_required(VERSION 2.8.3)
project(common_srvs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  message_generation
)

add_service_files(
  DIRECTORY
  srv
  FILES
  ConeMap.srv
  Path.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  common_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
