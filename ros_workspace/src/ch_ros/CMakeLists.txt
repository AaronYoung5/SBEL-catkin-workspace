cmake_minimum_required(VERSION 2.8.3)
project(ch_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  common_utilities
  common_msgs
  common_srvs
  roslaunch
)

find_package(Boost COMPONENTS program_options REQUIRED)

roslaunch_add_file_check(launch)

# Protobuf
find_package(Protobuf REQUIRED)

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
  src/protobuf_messages.pb.cc
)
target_link_libraries(ch_ros ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY} ${Boost_LIBRARIES})

install(PROGRAMS
  scripts/ch_launch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
