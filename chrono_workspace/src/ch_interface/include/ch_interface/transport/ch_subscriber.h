#pragma once

#include <ros/ros.h>

#include "ch_transport.h"
#include "ch_transport_type.h"

#include "ch_interface/flatbuffer/ch_flatbuffer_converter.h"

namespace chrono {
namespace transport {
class ChSubscriber : public ChTransport {
private:
  ros::Subscriber sub_;

public:
  ChSubscriber(ros::Subscriber &sub, TransportType type, std::string id,
               int freq);

  void spinOnce();
};
} // namespace transport
} // namespace chrono
