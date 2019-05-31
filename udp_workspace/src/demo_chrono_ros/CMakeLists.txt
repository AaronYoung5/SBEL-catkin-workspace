cmake_minimum_required(VERSION 2.8.3)
project(demo_chrono_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
)

roslaunch_add_file_check(launch)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/syn_server
  scripts/syn_vehicle
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
