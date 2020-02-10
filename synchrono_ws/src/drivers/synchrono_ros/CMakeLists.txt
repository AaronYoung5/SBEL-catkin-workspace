cmake_minimum_required(VERSION 2.8.3)
project(synchrono_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  common_msgs
  sensor_msgs
  image_transport
)

#-----------------------------------------------------------------------------
# Find MPI
#-----------------------------------------------------------------------------
find_package(MPI REQUIRED)
message(STATUS "MPI libraries: ${MPI_LIBRARIES}")

#-----------------------------------------------------------------------------
# Find Chrono
#-----------------------------------------------------------------------------
message(STATUS "Searching for Chrono...")
set(Chrono_DIR /home/aaron/chrono/chrono-ros/build/cmake/)
find_package(Chrono)

message(STATUS "Chrono libraries: ${CHRONO_LIBRARIES}")

# IF(Chrono_HAS_sensor)
# 	add_definitions(-DSENSOR_MODULE_ENABLED)
# ENDIF()

#-----------------------------------------------------------------------------
# Find Synchrono
#-----------------------------------------------------------------------------
message(STATUS "Searching for Synchrono...")
set(Synchrono_DIR /home/aaron/synchrono/synchrono-ros/build/cmake/)
find_package(Synchrono REQUIRED Interface CONFIG)

message(STATUS "Synchrono libraries: ${Synchrono_LIBRARIES}")

message("SYN Includes: ${Synchrono_INCLUDE_DIRS}")

# IF(Chrono_HAS_sensor)
# 	add_definitions(-DSENSOR_MODULE_ENABLED)
# ENDIF()

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
  ${CHRONO_INCLUDE_DIRS}
  ${Synchrono_INCLUDE_DIRS}
  ${MPI_C_HEADER_DIR}
  ${MPI_CXX_HEADER_DIR}
  ${FLATBUFFER_INCLUDE_DIRS}
)


catkin_package()

set(SYN_SOURCES
  src/main.cpp
  src/SynROSInterface.cpp
  src/SynROSTransportManager.cpp
)

add_executable(synchrono_ros ${SYN_SOURCES})
target_link_libraries(synchrono_ros ${catkin_LIBRARIES} ${Synchrono_LIBRARIES} ${FLATBUFFER_LIBRARY} ${MPI_LIBRARIES} ${CHRONO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(synchrono_ros ${catkin_EXPORTED_TARGETS})
set_target_properties(synchrono_ros PROPERTIES COMPILE_FLAGS "-lm")
