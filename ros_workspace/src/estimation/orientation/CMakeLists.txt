cmake_minimum_required(VERSION 2.8.3)
project(orientation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(

)

add_executable(orientation src/main.cpp)
target_link_libraries(orientation ${catkin_LIBRARIES})
