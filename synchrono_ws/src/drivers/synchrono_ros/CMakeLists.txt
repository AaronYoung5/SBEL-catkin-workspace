cmake_minimum_required(VERSION 2.8.3)
project(synchrono_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  common_msgs
  sensor_msgs
  image_transport
)

#-----------------------------------------------------------------------------
# Find Chrono
#-----------------------------------------------------------------------------
message(STATUS "Searching for Chrono...")
set(Chrono_DIR /home/aaron/chrono/chrono-ros/build/cmake/)
find_package(Chrono)

message(STATUS "Chrono libraries: ${CHRONO_LIBRARIES}")

#-----------------------------------------------------------------------------
# Find FlatBuffers
#-----------------------------------------------------------------------------
message(STATUS "Searching for FlatBuffers...")
find_package(Flatbuffers REQUIRED)
message(STATUS "${Flatbuffers_FOUND}")

message(STATUS "FlatBuffer libraries: ${FLATBUFFER_LIBRARY}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${FLATBUFFER_INCLUDE_DIRS}
  ${CHRONO_INCLUDE_DIRS}
)


catkin_package()

set(SYN_ROS_SOURCES
  src/main.cpp
  src/SynInterface.cpp
  src/SynTransportManager.cpp
  src/SynFlatBuffersManager.cpp
)

add_executable(synchrono_ros ${SYN_ROS_SOURCES})
target_link_libraries(synchrono_ros ${catkin_LIBRARIES} ${FLATBUFFER_LIBRARY} ${CHRONO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(synchrono_ros ${catkin_EXPORTED_TARGETS})
