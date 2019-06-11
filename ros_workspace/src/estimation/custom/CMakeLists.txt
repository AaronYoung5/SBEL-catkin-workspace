cmake_minimum_required(VERSION 2.8.3)
project(custom)

find_package(catkin REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(

)
