cmake_minimum_required(VERSION 2.8.3)
project(chrono)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_package(

)

add_executable(chrono
  src/main.cpp
  src/ch_transport_manager.cpp
  src/ch_interface.cpp
)
target_link_libraries(chrono ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(chrono ${catkin_EXPORTED_TARGETS})

install(PROGRAMS
  scripts/ch_launch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
