cmake_minimum_required(VERSION 2.8.3)
project(chrono_com)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
    CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
    include
)

add_executable(chrono_com src/chrono_com.cpp)
target_link_libraries(chrono_com ${catkin_LIBRARIES})
