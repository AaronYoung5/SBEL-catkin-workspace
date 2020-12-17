//ROS include
#include "ros/ros.h"

#include "syn_interface/syn_interface.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "syn_interface");
  ros::NodeHandle n("~");

  SynInterface syn_interface(n);

  syn_interface.Run();
}
