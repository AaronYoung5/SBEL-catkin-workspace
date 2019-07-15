cmake_minimum_required(VERSION 2.8.3)
project(dead_reckoning)

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

add_executable(dead_reckoning src/main.cpp)
target_link_libraries(dead_reckoning ${catkin_LIBRARIES})

add_dependencies(dead_reckoning ${catkin_EXPORTED_TARGETS})
