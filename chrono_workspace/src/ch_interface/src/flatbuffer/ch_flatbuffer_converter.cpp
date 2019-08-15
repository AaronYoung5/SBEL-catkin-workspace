#include "ch_interface/transport/ch_transport_type.h"

#include "ch_interface/flatbuffer/ch_flatbuffer_converter.h"

namespace chrono {
namespace flatbuffer {
ChFlatbufferConverter::ChFlatbufferConverter() {}

template <class msg_type>
msg_type
ChFlatbufferConverter::convert(TransportType type,
                               const ChInterfaceMessage::Message *message) {
  switch (type) {
  case TransportType::CAMERA:
    return toImage(
        static_cast<const ChInterfaceMessage::Camera *>(message->message()));
  case TransportType::TIME:
    return toTime(
        static_cast<const ChInterfaceMessage::Time *>(message->message()));
  }
}

sensor_msgs::Image
ChFlatbufferConverter::toImage(const ChInterfaceMessage::Camera *camera) {
  sensor_msgs::Image image;

  image.header.stamp = ros::Time::now();
  image.header.frame_id = "base_link";

  image.height = camera->height();
  image.width = camera->width();

  image.encoding = sensor_msgs::image_encodings::RGBA8;

  int num = 1; // for endianness detection
  image.is_bigendian = !(*(char *)&num == 1);

  image.step = camera->bytes_per_pixel() * image.width;

  size_t size = image.width * image.height * camera->bytes_per_pixel();
  image.data.resize(size);
  memcpy((uint8_t *)(&image.data[0]), camera->data()->Data(), size);

  return image;
}

rosgraph_msgs::Clock
ChFlatbufferConverter::toTime(const ChInterfaceMessage::Time *time) {
  rosgraph_msgs::Clock msg;

  msg.clock = ros::Time(time->t());

  return msg;
}
} // namespace flatbuffer
} // namespace chrono
