cmake_minimum_required(VERSION 2.8.3)
project(car_visualizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  common_msgs
  common_utilities
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(

)

add_executable(car_visualizer src/main.cpp)
target_link_libraries(car_visualizer ${catkin_LIBRARIES})
