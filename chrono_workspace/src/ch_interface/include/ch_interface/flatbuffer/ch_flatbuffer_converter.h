#pragma once

#include "ch_interface_messages_generated.h"

#include "rosgraph_msgs/Clock.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/image_encodings.h"

#include "common_msgs/Control.h"

#include "ch_interface/transport/ch_transport_type.h"

using namespace chrono::transport;

namespace chrono {
namespace flatbuffer {
static inline sensor_msgs::Image
toImage(const ChInterfaceMessage::Camera *camera) {
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

static inline rosgraph_msgs::Clock
toTime(const ChInterfaceMessage::Time *time) {
  rosgraph_msgs::Clock msg;

  msg.clock = ros::Time(time->t());

  return msg;
}

static inline void fromControl(const common_msgs::Control::ConstPtr &msg) {
  // flatbuffers::FlatBufferBuilder builder;
  //
  // flatbuffers::Offset<ChInterfaceMessage::Control> control =
  //     ChInterfaceMessage::CreateControl(builder, msg->throttle, msg->steering,
  //                                       msg->braking);
  // flatbuffers::Offset<ChInterfaceMessage::Message> message =
  //     ChInterfaceMessage::CreateMessage(
  //         builder, ChInterfaceMessage::Type_Control, control.Union());
  //
  // builder.FinishSizePrefixed(message);
  //
  // // Get size of FlatBuffer message in bytes
  // int32_t size = builder.GetSize();
  // // Get buffer pointer from message object
  // uint8_t *buffer = builder.GetBufferPointer();
  // // Send the message
  // socket_.async_send(boost::asio::buffer(buffer, size),
  //                    [&](const boost::system::error_code &ec, size_t size) {});
}
} // namespace flatbuffer
} // namespace chrono
