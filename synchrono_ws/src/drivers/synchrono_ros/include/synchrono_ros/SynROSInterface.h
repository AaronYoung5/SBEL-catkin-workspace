#pragma once

#include <ros/ros.h>

#include "synchrono_interface/SynInterface.h"
#include "synchrono/flatbuffers/SynFlatBuffersManager.h"
#include "synchrono_ros/transport/SynROSTransportManager.h"
#include "synchrono/SynApi.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace synchrono;

namespace synchrono {
namespace interface {
class SYN_API SynROSInterface : public SynInterface {
public:
  SynROSInterface(ros::NodeHandle &n, SynFlatBuffersManager &flatbuffers_manager);

  virtual void Synchronize(double ch_time);

protected:
  SynROSTransportManager m_ros_transport_manager;
};
} // namespace interface
} // namespace synchrono
