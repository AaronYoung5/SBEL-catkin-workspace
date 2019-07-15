cmake_minimum_required(VERSION 2.8.3)
project(path_visualizer)

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

add_executable(path_visualizer src/main.cpp)
target_link_libraries(path_visualizer ${catkin_LIBRARIES})

add_dependencies(path_visualizer ${catkin_EXPORTED_TARGETS})
