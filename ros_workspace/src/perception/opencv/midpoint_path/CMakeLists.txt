cmake_minimum_required(VERSION 2.8.3)
project(opencv_midpoint_path)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  common_msgs
  cv_bridge
  image_transport
)

find_package( OpenCV 3 REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

catkin_package(

)

add_executable(opencv_midpoint_path src/main.cpp src/image_converter.cpp)
target_link_libraries(opencv_midpoint_path ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(opencv_midpoint_path ${catkin_EXPORTED_TARGETS})
