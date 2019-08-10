cmake_minimum_required(VERSION 2.8.3)
project(video_streamer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  common_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(video_streamer src/main.cpp)
target_link_libraries(video_streamer ${catkin_LIBRARIES})
add_dependencies(video_streamer ${catkin_EXPORTED_TARGETS})
