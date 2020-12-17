#pragma once

#include <ros/serialization.h>

#include "ch_interface/flatbuffer/ch_interface_messages_generated.h"

#include "common_msgs/Control.h"

namespace chrono {
namespace flatbuffer {
class ChFlatbufferHandler {
private:
  std::vector<uint8_t> buffer_;

public:
  ChFlatbufferHandler() {}

  uint8_t *data() { return buffer_.data(); }
  void resize(int size) { buffer_.resize(size); }
  int getPrefixedSize() { return ((int *)buffer_.data())[0]; }

  // sensor_msgs::Image toImage() {
  //   sensor_msgs::Image image;
  //   image.header.stamp = ros::Time::now();
  //   image.header.frame_id = "base_link";
  //
  //   image.height = camera->height();
  //   image.width = camera->width();
  //
  //   image.encoding = sensor_msgs::image_encodings::RGBA8;
  //
  //   int num = 1; // for endianness detection
  //   image.is_bigendian = !(*(char *)&num == 1);
  //
  //   image.step = camera->bytes_per_pixel() * image.width;
  //
  //   size_t size = image.width * image.height * camera->bytes_per_pixel();
  //   image.data.resize(size);
  //   memcpy((uint8_t *)(&image.data[0]), camera->data()->Data(), size);
  //
  //   return image;
  // }

  void controlCallback(const common_msgs::Control::ConstPtr &msg) {
    flatbuffers::FlatBufferBuilder builder;

    flatbuffers::Offset<ChInterfaceMessage::Control> control =
        ChInterfaceMessage::CreateControl(builder, msg->throttle, msg->steering,
                                          msg->braking);
    flatbuffers::Offset<ChInterfaceMessage::Message> message =
        ChInterfaceMessage::CreateMessage(
            builder, ChInterfaceMessage::Type_Control, control.Union());
  }

  template <typename T>
  void SerializeToByteArray(const T &msg, std::vector<uint8_t> &msg_buffer) {
    const uint32_t length = ros::serialization::serializationLength(msg);
    msg_buffer.resize(length);
    // copy into your own buffer
    ros::serialization::OStream stream(msg_buffer.data(), length);
    ros::serialization::serialize(stream, msg);
  }
};
} // namespace flatbuffer
} // namespace chrono
