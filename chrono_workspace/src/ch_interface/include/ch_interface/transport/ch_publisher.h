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

  ChFlatbufferConverter fb_converter_;

public:
  ChPublisher(ros::Publisher &pub, std::string id, int freq);

  template <class msg_type>
  void operator()(TransportType type,
                  const ChInterfaceMessage::Message *message);
};
} // namespace transport
} // namespace chrono
