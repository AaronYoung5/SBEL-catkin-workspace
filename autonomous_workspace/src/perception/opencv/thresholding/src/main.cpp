// ROS include
#include "ros/ros.h"
#include "thresholding/thresholder.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "thresholding");
  ros::NodeHandle n("~");

  Thresholder converter(n);

  ros::spin();
}
