cmake_minimum_required(VERSION 2.8.3)
project(chrono_ros_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  genmsg
  geometry_msgs
)

# Protobuf
find_package(Protobuf REQUIRED)

# add_custom_command(
#   OUTPUT ProtobufMessages.pb.cc ProtobufMessages.pb.h
#   COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} --
#   cpp_out=${CMAKE_CURRENT_BINARY_DIR} ProtobufMessages.proto --cpp_out=.
#   WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/lib/"
# )

add_message_files(
  DIRECTORY
  msg
  FILES
  Cones.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp geometry_msgs message_runtime
    INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(LIBS
  ChROS/MessageHandler.cpp
  ChROS/ProtobufMessages/ProtobufMessages.pb.cc
  ChROS/MessagePublisher.cpp
)

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} LIBS ${PROTOBUF_LIBRARY})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
