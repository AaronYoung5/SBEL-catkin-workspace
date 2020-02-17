#pragma once

#include <ros/ros.h>

#include <map>

#include "synchrono_ros/components/SynComponent.h"
#include "synchrono_ros/flatbuffers/SynFlatBuffersManager.h"
#include "synchrono_ros/transport/SynTransportManager.h"

class SynInterface {
public:
  SynInterface(ros::NodeHandle &n);

  void Synchronize();

  void AddComponent(std::shared_ptr<SynComponent> component);

private:
  std::map<std::string, std::shared_ptr<SynComponent>> m_components;

  SynFlatBuffersManager m_flatbuffers_manager;
  SynTransportManager m_transport_manager;
};
