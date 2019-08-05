cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  common_utilities
  common_msgs
)

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(kalman_filter src/main.cpp src/kalman_estimation.cpp)
target_link_libraries(kalman_filter ${catkin_LIBRARIES})
