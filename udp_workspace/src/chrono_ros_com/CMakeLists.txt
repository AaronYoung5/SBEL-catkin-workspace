cmake_minimum_required(VERSION 2.8.3)
project(chrono_ros_com)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  genmsg
)

# Protobuf
find_package(Protobuf REQUIRED)

add_custom_command(
  OUTPUT ProtobufMessages.pb.cc ProtobufMessages.pb.h
  COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} --
  cpp_out=${CMAKE_CURRENT_BINARY_DIR} ProtobufMessages.proto --cpp_out=.
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/lib/"
)

catkin_package(
    CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(LIBRARIES
  lib/MessageHandler.cpp
  lib/ProtobufMessages.pb.cc
  lib/LidarPublisher.cpp
  lib/GPSPublisher.cpp
  lib/IMUPublisher.cpp
  lib/ConePublisher.cpp
)

add_executable(chrono_ros_com src/chrono_ros_com.cpp)
target_link_libraries(chrono_ros_com ${catkin_LIBRARIES} LIBRARIES ${PROTOBUF_LIBRARY})
