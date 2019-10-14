cmake_minimum_required(VERSION 2.8.3)
project(common_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

add_message_files(
  DIRECTORY
  msg
  FILES
  Control.msg
)

generate_messages(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
