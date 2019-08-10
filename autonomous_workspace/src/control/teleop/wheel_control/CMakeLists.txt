cmake_minimum_required(VERSION 2.8.3)
project(wheel_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  common_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(

)

add_executable(wheel_control src/main.cpp)
target_link_libraries(wheel_control ${catkin_LIBRARIES})

add_dependencies(wheel_control ${catkin_EXPORTED_TARGETS})
