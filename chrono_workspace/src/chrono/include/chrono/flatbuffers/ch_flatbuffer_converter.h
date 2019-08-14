#include "ch_interface_message_generated.fbs"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"

namespace chrono {
namespace flatbuffer_converter {
namespace flatbuffer_utilities {
enum FlatbufferDataTypes {
  INT8,
  UINT8,
  INT16,
  UINT16,
  INT32,
  UINT32,
  FLOAT32,
  FLOAT64
};

int8_t getBytesPerValue(int data_type) {
  switch (data_type) {
  case FlatbufferDataTypes::INT8:
  case FlatbufferDataTypes::UINT8:
    return 8;
  case FlatbufferDataTypes::INT16:
  case FlatbufferDataTypes::UINT16:
    return 16;
  case FlatbufferDataTypes::INT32:
  case FlatbufferDataTypes::UINT32:
  case FlatbufferDataTypes::FLOAT32:
    return 32;
  case FlatbufferDataTypes::FLOAT64:
    return 64;
  }
}

uint8_t getPointCloudDataType(int data_type) { return (uint8_t)data_type }
} // namespace flatbuffer_utilities

sensor_msgs::Image convertToImage(const ChInterfaceMessage::Message *message) {
  const ChInterfaceMessage::Camera *camera =
      static_cast<const ChInterfaceMessage::Camera *>(message->message());

  sensor_msgs::Image image;

  image.header.stamp = ros::Time::now();
  image.header.frame_id = "base_link";

  image.height = camera->height();
  image.width = camera->width();

  image.encoding = sensor_msgs::image_encodings::RGBA8;

  int num = 1; // for endianness detection
  image.is_bigendian = !(*(char *)&num == 1);

  image.step = flatbuffer_utilities::getBytesPerValue(message->data_type()) *
               data_.width;

  size_t len = image.height * image.step;

  image.data.resize(len);
  memcpy((uint8_t *)(&image.data[0]), message->data()->Data(), len);

  return image;
}

sensor_msgs::PointCloud2 &
convertToPointCloud2(const ChInterfaceMessage::Message *message) {
  const ChRosMessage::Lidar *lidar =
      static_cast<const ChRosMessage::Lidar *>(message->message());

  sensor_msgs::PointCloud2 point_cloud;

  int bytes_per_value =
      flatbuffer_utilities::getBytesPerValue(message->data_type());
  int data_type =
      flatbuffer_utilities::getPointCloudDataType(message->data_type());

  point_cloud.header.stamp = ros::Time::now();
  point_cloud.header.frame_id = "base_link";

  point_cloud.width = lidar->height() * lidar->width() * bytes_per_value;
  point_cloud.height = 1;

  // Convert x/y/z to fields
  int num_fields = lidar->fields()->Length();
  point_cloud.fields.resize(num_fields);
  for (int i = 0; i < num_fields; i++) {

    point_cloud.fields[i].name = (char *)(lidar->fields()[i]);
  }

  int offset = 0;
  for (size_t d = 0; d < num_fields; d++, offset += num_fields) {
    point_cloud.fields[d].offset = offset;
    point_cloud.fields[d].datatype = data_type;
    point_cloud.fields[d].count = 1;
  }

  point_cloud.point_step = offset;
  point_cloud.row_step = point_cloud.point_step * point_cloud.width;

  point_cloud.data.resize(point_cloud.row_step);
  int num = 1; // for endianness detection
  point_cloud.is_bigendian = !(*(char *)&num == 1);
  point_cloud.is_dense = false;

  memcpy((uint8_t *)(&point_cloud.data[0]), message->data()->Data(),
         point_cloud.width);

  return point_cloud;
}

sensor_msgs::NavSatFix &
convertToNavSatFix(const ChInterfaceMessage::Message *message) {
  // Convert to NavSatFix
  sensor_msgs::NavSatFix gps;

  gps.header.stamp = ros::Time::now();
  gps.header.frame_id = "map";

  int bytes_per_value =
      flatbuffer_utilities::getBytesPerValue(message->data_type());

  gps.latitude = message->data()->Data()[0];
  gps.longitude = message->data()->Data()[bytes_per_pixel];
  gps.altitude = message->data()->Data()[bytes_per_pixel * 2];

  return gps;
}

sensor_msgs::Imu &convertToImu(const ChInterfaceMessage::Message *message) {
  // Convert to Imu
  sensor_msgs::Imu imu;

  imu.header.stamp = ros::Time::now();
  imu.header.frame_id = "base_link";

  int bytes_per_value =
      flatbuffer_utilities::getBytesPerValue(message->data_type());

  int index = 0;
  imu.orientation.w = message->data()->Data()[index];
  imu.orientation.x = message->data()->Data()[index + bytes_per_value];
  imu.orientation.y = message->data()->Data()[index + bytes_per_value * 2];
  imu.orientation.z = message->data()->Data()[index + bytes_per_value * 3];

  index += bytes_per_pixel * 4;

  imu.linear_acceleration.x = message->data()->Data()[index];
  imu.linear_acceleration.y = message->data()->Data()[index + bytes_per_value];
  imu.linear_acceleration.z =
      message->data()->Data()[index + bytes_per_value * 2];

  index += bytes_per_pixel * 3;

  imu.angular_velocity.x = message->data()->Data()[index];
  imu.angular_velocity.y = message->data()->Data()[index + bytes_per_value];
  imu.angular_velocity.z = message->data()->Data()[index + bytes_per_value * 2];

  return imu;
}
} // namespace flatbuffer_converter
} // namespace chrono
