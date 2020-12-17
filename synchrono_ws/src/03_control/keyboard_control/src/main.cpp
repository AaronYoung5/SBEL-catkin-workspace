#include <ros/ros.h>

#include <common_msgs/Control.h>

#include "keyboard_control/controller.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "keyboard_control");
  ros::NodeHandle n;

  Controller controller(n);

  controller.keyLoop();
}
