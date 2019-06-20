// ROS includes
#include "ros/ros.h"

// Internal package includes
#include "ch_ros/ch_ros_handler.h"

#define TCP

void exit(int signal) {
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ch_ros");
  ros::NodeHandle n;

  ChRosHandler handler(n);

  signal(SIGINT, exit);

  while(ros::ok() && handler.ok()) {
    #ifdef TCP
    handler.tcpReceiveAndHandle();
    #else
    handler.receiveAndHandle();
    #endif

    ros::spinOnce();
  }

  ros::shutdown();
}
