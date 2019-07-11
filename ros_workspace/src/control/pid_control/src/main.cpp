// ROS include
#include "ros/ros.h"

// Header file includes
#include "pid_control/pid_control.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pid_control");
  ros::NodeHandle n;

  PIDControl pid(n);

  ros::spin();
}
