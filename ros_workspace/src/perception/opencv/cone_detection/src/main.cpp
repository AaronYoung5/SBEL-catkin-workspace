// ROS include
#include "ros/ros.h"
#include "cone_detection/image_converter.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "opencv_cone_detection");
  ros::NodeHandle n;

  ImageConverter converter(n);

  ros::spin();
}
