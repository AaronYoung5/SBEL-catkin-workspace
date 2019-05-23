cmake_minimum_required(VERSION 2.8.3)
project(chrono_com)

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
)

add_executable(server src/server.cpp)
target_link_libraries(server ${catkin_LIBRARIES})

add_executable(client src/client.cpp)
target_link_libraries(client ${catkin_LIBRARIES})
