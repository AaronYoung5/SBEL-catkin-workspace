cmake_minimum_required(VERSION 2.8.3)
project(chrono_ros)

# Ros packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    message_generation
    genmsg
    roslaunch
)

roslaunch_add_file_check(launch)

# Sets chronos cmake directory
set(Chrono_DIR /home/aaron/chrono/build/cmake)

# Chrono packages
find_package(Chrono REQUIRED COMPONENTS
  Irrlicht
  Vehicle
  CONFIG
)

## Generate messages in the 'msg' folder
# add_message_files(
#     DIRECTORY
#     msg
#     FILES
#
# )

## Generate services in the 'srv' folder
# add_service_files (
#   DIRECTORY
#   srv
#   FILES
#
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#     DEPENDENCIES
#     std_msgs
#     geometry_msgs
# )

catkin_package(
    CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${CHRONO_INCLUDE_DIRS}
    include
)

## Declare a library for Chrono implementation
add_library(chrono_ros_library
  chrono_ros_launcher.cpp
)
## Set target properties of Chrono library with correct data directory
set_target_properties(chrono_ros_library
  PROPERTIES
  COMPILE_DEFINITIONS
  "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
)
## Link Chrono libraries to Chrono/ROS implementation
target_link_libraries(chrono_ros_library ${CHRONO_LIBRARIES})

add_executable(chrono_ros src/chrono_ros.cpp)
target_link_libraries(chrono_ros ${catkin_LIBRARIES} chrono_ros_library)
