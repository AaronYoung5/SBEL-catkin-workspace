#pragma once

#include <ros/ros.h>

#include <common_msgs/Control.h>

#include "syn_interface/components/syn_camera_component.h"
#include "syn_interface/components/syn_control_component.h"
#include "syn_interface/components/syn_interface_component.h"
#include "syn_interface/components/syn_time_component.h"
#include "syn_interface/components/syn_lidar_component.h"
#include "syn_interface/syn_flatbuffers_manager.h"
#include "syn_interface/syn_transport_manager.h"

class SynInterface {
private:
  SynTransportManager transport_manager_;

  SynFlatBuffersManager flatbuffers_manager_;

  std::vector<std::string> names_;
  std::vector<std::shared_ptr<SynInterfaceComponent>> components_;

public:
  SynInterface(ros::NodeHandle &n);

  void Run();
  void Add(std::shared_ptr<SynInterfaceComponent> component);
};
