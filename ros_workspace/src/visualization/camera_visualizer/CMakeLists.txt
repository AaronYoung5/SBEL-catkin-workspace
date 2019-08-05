cmake_minimum_required(VERSION 2.8.3)
project(camera_visualizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
)

find_package( OpenCV 3 REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(

)

add_executable(camera_visualizer src/main.cpp)
target_link_libraries(camera_visualizer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_dependencies(camera_visualizer ${catkin_EXPORTED_TARGETS})
