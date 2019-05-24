cmake_minimum_required(VERSION 2.8.3)
project(udp_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
)

roslaunch_add_file_check(launch)

catkin_package(
    CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
    include
)

# add_library(log_lib
#   lib/log.cpp
# )

add_library(udp_lib
  lib/client.cpp
  lib/server.cpp
)

# target_link_libraries(udp_lib log_lib)

add_executable(udp_server src/udp_server.cpp)
target_link_libraries(udp_server ${catkin_LIBRARIES} udp_lib)

add_executable(udp_client src/udp_client.cpp)
target_link_libraries(udp_client ${catkin_LIBRARIES} udp_lib)

# add_executable(server src/server.cpp)
# target_link_libraries(server ${catkin_LIBRARIES} udp_lib)
#
# add_executable(client src/client.cpp)
# target_link_libraries(client ${catkin_LIBRARIES} udp_lib)
