cmake_minimum_required(VERSION 2.8.3)
project(thresholding)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  perception_msgs
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

add_executable(thresholding src/main.cpp src/thresholder.cpp)
target_link_libraries(thresholding ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(thresholding ${catkin_EXPORTED_TARGETS})
