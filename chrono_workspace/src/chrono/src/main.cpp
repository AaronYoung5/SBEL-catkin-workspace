#include <ros/ros.h>

#include "chrono/ch_interface.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "chrono");
  ros::NodeHandle n;

  ChInterface ch_interface(n);

  ch_interface.run();
}
