// ROS include
#include "ros/ros.h"

// Internal package includes
#include "ch_ros/ch_ros_handler.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ch_ros");
  ros::NodeHandle n;

  ChRosHandler handler(n);

  while (ros::ok() && handler.chrono_ok()) {
    handler.receive();
    ros::spinOnce();
  }

  ros::shutdown();
}
