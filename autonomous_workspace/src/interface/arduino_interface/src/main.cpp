// ROS include
#include "ros/ros.h"

// Package includes
#include "arduino_interface/serial_handler.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "arduino_interface");
  ros::NodeHandle n;

  SerialHandler serialHandler(n);

  return serialHandler.spin();
}
