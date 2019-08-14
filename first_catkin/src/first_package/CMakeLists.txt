cmake_minimum_required(VERSION 2.8.3)
project(first_package)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  genmsg
)

add_message_files(
  DIRECTORY
  msg
  FILES
  Num.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
