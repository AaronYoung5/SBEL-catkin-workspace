cmake_minimum_required(VERSION 2.8.3)
project(control_utilities)

find_package(catkin REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
)

install(DIRECTORY /include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
