// #include "ch_interface/flatbuffer/ch_flatbuffer_converter.h"
//
// namespace chrono {
// namespace flatbuffer {
// inline sensor_msgs::Image toImage(const ChInterfaceMessage::Camera *camera) {
//   sensor_msgs::Image image;
//
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
//
// inline rosgraph_msgs::Clock toTime(const ChInterfaceMessage::Time *time) {
//   rosgraph_msgs::Clock msg;
//
//   msg.clock = ros::Time(time->t());
//
//   return msg;
// }
// } // namespace flatbuffer
// } // namespace chrono
