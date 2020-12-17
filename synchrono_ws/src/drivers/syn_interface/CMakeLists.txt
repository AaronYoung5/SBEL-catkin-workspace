cmake_minimum_required(VERSION 2.8.3)
project(syn_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  common_msgs
  sensor_msgs
  image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(syn_interface src/main.cpp src/syn_interface.cpp src/syn_transport_manager.cpp src/syn_flatbuffers_manager.cpp)
target_link_libraries(syn_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(syn_interface ${catkin_EXPORTED_TARGETS})
