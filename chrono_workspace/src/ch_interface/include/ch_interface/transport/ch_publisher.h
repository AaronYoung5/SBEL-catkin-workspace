#pragma once

#include <ros/ros.h>

#include "ch_transport.h"
#include "ch_transport_type.h"

#include "ch_interface/flatbuffer/ch_flatbuffer_handler.h"

using namespace chrono::flatbuffer;

namespace chrono {
namespace transport {
class ChPublisher : public ChTransport {
private:
  ros::Publisher pub_;

public:
  ChPublisher(ros::Publisher &pub, TransportType type, std::string id,
              int freq);

  void spinOnce(ChFlatbufferHandler &flatbuffer_handler);
};
} // namespace transport
} // namespace chrono
