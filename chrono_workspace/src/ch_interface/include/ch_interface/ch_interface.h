#pragma once

#include <ros/ros.h>

#include "ch_interface/transport/ch_transport_manager.h"

using namespace chrono::transport;

namespace chrono {
class ChInterface {
private:
  ChTransportManager transport_manager_;

public:
  ChInterface(ros::NodeHandle &n);

  ~ChInterface();

  void run();
};
} // namespace chrono
