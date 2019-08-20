cmake_minimum_required(VERSION 2.8.3)
project(ch_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  rosgraph_msgs
  common_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_package(

)

file(GLOB transport_SRC "src/transport/*")

file(GLOB flatbuffer_SRC "src/flatbuffer/*")

add_executable(ch_interface
  src/main.cpp
  src/ch_interface.cpp
  ${transport_SRC}
  ${flatbuffer_SRC}
)
target_link_libraries(ch_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(ch_interface ${catkin_EXPORTED_TARGETS})

install(PROGRAMS
  scripts/ch_launch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
