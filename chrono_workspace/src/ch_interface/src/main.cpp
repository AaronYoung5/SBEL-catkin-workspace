#include <ros/ros.h>

#include "ch_interface/ch_interface.h"

using namespace chrono;

int main(int argc, char **argv) {
  ros::init(argc, argv, "ch_interface");
  ros::NodeHandle n;

  ChInterface ch_interface(n);

  ch_interface.run();
}
