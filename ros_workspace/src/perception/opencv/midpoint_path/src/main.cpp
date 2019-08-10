// ROS include
#include "ros/ros.h"
#include "midpoint_path/image_converter.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "opencv_midpoint_path");
  ros::NodeHandle n;

  ImageConverter converter(n);

  ros::spin();
}
