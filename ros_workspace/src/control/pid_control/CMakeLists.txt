cmake_minimum_required(VERSION 2.8.3)
project(pid_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  common_msgs
  common_utilities
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(

)

add_executable(pid_control src/main.cpp src/PID.cpp src/pid_control.cpp)
target_link_libraries(pid_control ${catkin_LIBRARIES})
