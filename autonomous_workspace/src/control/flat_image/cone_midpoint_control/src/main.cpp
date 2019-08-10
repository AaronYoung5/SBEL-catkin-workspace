// ROS include
#include "ros/ros.h"
#include "cone_midpoint_control/controller.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "cone_midpoint");
  ros::NodeHandle n("~");

  Controller controller(n);

  ros::spin();
}
