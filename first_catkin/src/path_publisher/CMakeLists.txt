cmake_minimum_required(VERSION 2.8.3)
project(path_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  genmsg
  geometry_msgs
)

add_message_files(
  DIRECTORY
  msg
  FILES
  path_msg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_publisher src/path_publisher.cpp)
target_link_libraries(path_publisher ${catkin_LIBRARIES})
