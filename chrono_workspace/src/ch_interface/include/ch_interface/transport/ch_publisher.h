#pragma once

#include <ros/ros.h>

#include "ch_transport.h"
#include "ch_transport_type.h"

#include "ch_interface/flatbuffer/ch_flatbuffer_converter.h"

using namespace chrono::flatbuffer;

namespace chrono {
namespace transport {
class ChPublisher : public ChTransport {
private:
  ros::Publisher pub_;

  const ChInterfaceMessage::Message *message_;

public:
  ChPublisher(ros::Publisher &pub, TransportType type, std::string id, int freq);

  void update(const ChInterfaceMessage::Message *message);
  void spinOnce();
};
} // namespace transport
} // namespace chrono
