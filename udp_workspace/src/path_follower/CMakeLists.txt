cmake_minimum_required(VERSION 2.8.3)
project(path_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  chrono_ros_interface
)

catkin_package(
  
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pid_follower src/pid_follower.cpp)
target_link_libraries(pid_follower ${catkin_LIBRARIES})
