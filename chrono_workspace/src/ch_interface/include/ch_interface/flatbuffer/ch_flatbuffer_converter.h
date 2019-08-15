#pragma once

#include "ch_interface_messages_generated.h"

#include "rosgraph_msgs/Clock.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"

#include "ch_interface/transport/ch_transport_type.h"

using namespace chrono::transport;

namespace chrono {
namespace flatbuffer {
class ChFlatbufferConverter {
private:
public:
  ChFlatbufferConverter();

  template <class msg_type>
  msg_type convert(TransportType type,
                   const ChInterfaceMessage::Message *message);

private:
  sensor_msgs::Image toImage(const ChInterfaceMessage::Camera *camera);
  rosgraph_msgs::Clock toTime(const ChInterfaceMessage::Time *time);
};
} // namespace flatbuffer
} // namespace chrono
