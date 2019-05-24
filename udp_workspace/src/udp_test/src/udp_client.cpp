#include "ros/ros.h"
#include "udp_test/client.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "udp_test");

  Client client;

  while(ros::ok()) {
    client.HandleMsgs();
  }

  client.Close();
}
