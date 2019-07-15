cmake_minimum_required(VERSION 2.8.3)
project(ekf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(

)

add_executable(ekf src/main.cpp)
target_link_libraries(ekf ${catkin_LIBRARIES})

add_dependencies(ekf ${catkin_EXPORTED_TARGETS})
