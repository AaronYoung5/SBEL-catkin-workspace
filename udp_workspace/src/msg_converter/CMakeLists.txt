cmake_minimum_required(VERSION 2.8.3)
project(msg_converter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  chrono_ros_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(

)

add_executable(gps_to_pose src/gps_to_pose.cpp)
target_link_libraries(gps_to_pose ${catkin_LIBRARIES})

add_executable(cones_to_markers src/cones_to_markers.cpp)
target_link_libraries(cones_to_markers ${catkin_LIBRARIES})

add_executable(gps_to_odom src/gps_to_odom.cpp)
target_link_libraries(gps_to_odom ${catkin_LIBRARIES})
