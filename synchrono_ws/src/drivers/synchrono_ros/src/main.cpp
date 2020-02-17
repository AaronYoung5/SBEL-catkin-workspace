// ROS include
#include <ros/ros.h>

#include "synchrono_ros/SynInterface.h"

#include "synchrono_ros/components/SynCameraComponent.h"
#include "synchrono_ros/components/SynControlComponent.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "synchrono_ros");
  ros::NodeHandle n("~");

  SynInterface syn_interface(n);
  syn_interface.AddComponent(
      std::make_shared<SynCameraComponent>(n, "/camera"));
  syn_interface.AddComponent(
      std::make_shared<SynControlComponent>(n, "/control"));

  while (ros::ok()) {
    syn_interface.Synchronize();
  }
}
