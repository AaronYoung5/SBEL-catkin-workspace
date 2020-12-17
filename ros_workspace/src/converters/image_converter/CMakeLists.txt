cmake_minimum_required(VERSION 2.8.3)
project(image_converter)

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

install(PROGRAMS
  scripts/bag_to_jpg
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
