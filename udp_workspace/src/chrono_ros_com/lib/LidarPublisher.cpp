#include "chrono_ros_com/LidarPublisher.h"

LidarPublisher::LidarPublisher(ros::NodeHandle n, MessageHandler &handler)
    : m_handler(handler), m_n(n) {
  m_pub = m_n.advertise<sensor_msgs::PointCloud>("cloud", 1000);
  m_pub2 = m_n.advertise<sensor_msgs::PointCloud2>("cloud2", 1000);

  m_cloud.header.frame_id = "cloud";
  m_cloud.header.seq = 0;

  m_cloud2.header.frame_id = "cloud2";
  m_cloud2.header.seq = 0;
}

void LidarPublisher::PublishPointCloud2() {
  std::vector<Position> data = m_handler.LidarData();

  m_cloud2.header.stamp = ros::Time::now();
  m_cloud2.header.seq += 1;

  m_cloud2.width = data.size();
  m_cloud2.height = 1;

  // Convert x/y/z to fields
  m_cloud2.fields.resize(3);
  m_cloud2.fields[0].name = "x";
  m_cloud2.fields[1].name = "y";
  m_cloud2.fields[2].name = "z";

  int offset = 0;
  for (size_t d = 0; d < m_cloud2.fields.size(); d++, offset += 4) {
    m_cloud2.fields[d].offset = offset;
    m_cloud2.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
    m_cloud2.fields[d].count = 1;
  }

  // std::cout << offset << std::endl;
  m_cloud2.point_step = offset;
  m_cloud2.row_step = m_cloud2.point_step * m_cloud2.width;

  // for (size_t d = 0; d < data.
  m_cloud2.data.resize(data.size() * m_cloud2.point_step);
  m_cloud2.is_bigendian = false;
  m_cloud2.is_dense = false;

  for (size_t cp = 0; cp < data.size(); cp++) {
    memcpy(&m_cloud2.data[cp * m_cloud2.point_step + m_cloud2.fields[0].offset],
           &data[cp].x, sizeof(float));
    memcpy(&m_cloud2.data[cp * m_cloud2.point_step + m_cloud2.fields[1].offset],
           &data[cp].y, sizeof(float));
    memcpy(&m_cloud2.data[cp * m_cloud2.point_step + m_cloud2.fields[2].offset],
           &data[cp].z, sizeof(float));

    // for (
  }
  m_pub.publish(m_cloud2);
}

void LidarPublisher::PublishPointCloud() {
  std::vector<Position> data = m_handler.LidarData();

  m_cloud.header.stamp = ros::Time::now();
  m_cloud.header.seq += 1;

  m_cloud.points.resize(data.size());
  m_cloud.channels.resize(1);
  m_cloud.channels[0].name = "intensity";
  for (int i = 0; i < data.size(); i++) {
    m_cloud.points[i].x = data[i].x;
    m_cloud.points[i].y = data[i].y;
    m_cloud.points[i].z = data[i].z;

    m_cloud.channels[0].values.resize(data.size());
    m_cloud.channels[0].values[i] = i;
  }

  m_pub.publish(m_cloud);
}

void LidarPublisher::PublishConvertedPCToPC2() {
  std::vector<Position> data = m_handler.LidarData();

  m_cloud.header.stamp = ros::Time::now();
  m_cloud.header.seq += 1;

  m_cloud.points.resize(data.size());
  m_cloud.channels.resize(1);
  m_cloud.channels[0].name = "intensity";
  for (int i = 0; i < data.size(); i++) {
    m_cloud.points[i].x = data[i].x;
    m_cloud.points[i].y = data[i].y;
    m_cloud.points[i].z = data[i].z;

    m_cloud.channels[0].values.resize(data.size());
    m_cloud.channels[0].values[i] = i;
  }

  convertPointCloudToPointCloud2(m_cloud, m_cloud2);
  m_pub.publish(m_cloud2);
}
