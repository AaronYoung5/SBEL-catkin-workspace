cmake_minimum_required(VERSION 2.8.3)
project(basic_shape_tutorial)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(basic_shapes src/basic_shapes.cpp)
target_link_libraries(basic_shapes ${catkin_LIBRARIES})
