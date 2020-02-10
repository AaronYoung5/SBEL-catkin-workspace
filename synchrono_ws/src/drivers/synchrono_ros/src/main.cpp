//ROS include
#include "ros/ros.h"

#include "synchrono_ros/SynROSInterface.h"
#include "synchrono/flatbuffers/SynFlatBuffersManager.h"

using namespace synchrono::interface;

int main(int argc, char **argv) {
  ros::init(argc, argv, "syn_interface");
  ros::NodeHandle n("~");

  SynFlatBuffersManager manager;
  SynROSInterface syn_interface(n, manager);

  syn_interface.Synchronize(1);
}
