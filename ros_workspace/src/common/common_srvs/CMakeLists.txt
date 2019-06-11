cmake_minimum_required(VERSION 2.8.3)
project(common_srvs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  message_generation
)

# add_service_files(
#   DIRECTORY
#   srv
#   FILES
#   Control.srv
# )
#
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
