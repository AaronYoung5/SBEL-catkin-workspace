cmake_minimum_required(VERSION 2.8.3)
project(keyboard_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  common_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(

)

add_executable(keyboard_control src/main.cpp)
target_link_libraries(keyboard_control ${catkin_LIBRARIES})

add_dependencies(keyboard_control ${catkin_EXPORTED_TARGETS})
