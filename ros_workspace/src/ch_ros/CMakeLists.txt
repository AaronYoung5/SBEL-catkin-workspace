cmake_minimum_required(VERSION 2.8.3)
project(ch_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  common_utilities
  common_msgs
  common_srvs
  image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_package(

)

add_executable(ch_ros
  src/ch_ros.cpp
  src/ch_ros_handler.cpp
  src/publishers.cpp
)
target_link_libraries(ch_ros ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(PROGRAMS
  scripts/ch_launch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_dependencies(ch_ros ${catkin_EXPORTED_TARGETS})
