#include "ros/ros.h"
#include "udp_test/server.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "udp_test");

  Server server;

  while(ros::ok()) {
    server.HandleMsgs();
  }

  server.Close();
}
