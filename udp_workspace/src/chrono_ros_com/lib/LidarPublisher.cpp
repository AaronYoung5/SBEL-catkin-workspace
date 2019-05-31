#include "chrono_ros_com/LidarPublisher.h"

LidarPublisher::LidarPublisher(int argc, char **argv, MessageHandler &handler)
    : m_handler(handler) {
  ros::init(argc, argv, "LidarData");
  m_pub = n.advertise<sensor_msgs::PointCloud2>("laser", 1000);
  m_pub_cloud = n.advertise<sensor_msgs::PointCloud>("cloud", 1000);

  m_cloud.header.frame_id = "laser";
  m_cloud.header.seq = 0;

  cloud.header.frame_id = "cloud";
  cloud.header.seq = 0;
}

// void LidarPublisher::Publish(std::vector<Position> data) {
//   m_cloud.header.stamp = ros::Time::now();
//   m_cloud.header.seq += 1;
//
//   m_cloud.height = 1;
//   m_cloud.width = data.size();
//   m_cloud.fields.resize(data.size());
//   for (int i = 0; i < data.size(); i++) {
//     m_cloud.fields[i].name = i;
//     m_cloud.fields[i].offset = i * sizeof(double);
//     m_cloud.fields[i].count = 1;
//     m_cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
//   }
//
//   m_cloud.is_bigendian = false;
//   m_cloud.point_step = data.size() * sizeof(double);
//   m_cloud.row_step = data.size() * sizeof(double);
//   m_cloud.data.resize(m_cloud.row_step * m_cloud.height);
//
//   float *valPtr = (float *)(&(m_cloud.data[0]));
//   int off = 0;
//   std::vector<float> valSingle;
//   valSingle.resize(data.size());
//   for (int i = 0; i < data.size(); i++) {
//     valSingle[0] = data[i].x;
//     valSingle[1] = data[i].y;
//     valSingle[2] = data[i].z;
//     // valSingle switch (iLoop) {
//     //   case 0: {
//     //     float angle = deg2rad * rawTargetList[i].Azimuth();
//     //     valSingle[0] = rawTargetList[i].Dist() * cos(angle);
//     //     valSingle[1] = rawTargetList[i].Dist() * sin(angle);
//     //     valSingle[2] = 0.0;
//     //     valSingle[3] = rawTargetList[i].Vrad();
//     //     valSingle[4] = rawTargetList[i].Ampl();
//     //   } break;
//     //
//     //   case 1:
//     //     valSingle[0] = objectList[i].P3Dx();
//     //     valSingle[1] = objectList[i].P3Dy();
//     //     valSingle[2] = 0.0;
//     //     valSingle[3] = objectList[i].V3Dx();
//     //     valSingle[4] = objectList[i].V3Dy();
//     //     valSingle[5] = 0.0;
//     //     valSingle[6] = objectList[i].ObjLength();
//     //     valSingle[7] = objectList[i].ObjId();
//     //     break;
//     //   }
//     //
//     for (int j = 0; j < 1; j++) {
//       valPtr[off] = valSingle[j];
//       off++;
//     }
//   }
//
//   m_pub.publish(m_cloud);
// }

void LidarPublisher::Publish(std::vector<Position> data) {
  m_cloud.header.stamp = ros::Time::now();
  m_cloud.header.seq += 1;

  m_cloud.width = data.size();
  m_cloud.height = 1;

  // Convert x/y/z to fields
  m_cloud.fields.resize(3);
  m_cloud.fields[0].name = "x";
  m_cloud.fields[1].name = "y";
  m_cloud.fields[2].name = "z";

  int offset = 0;
  for (size_t d = 0; d < m_cloud.fields.size(); d++, offset += 4) {
    m_cloud.fields[d].offset = offset;
    m_cloud.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
    m_cloud.fields[d].count = 1;
  }

  m_cloud.point_step = offset;
  m_cloud.row_step = m_cloud.point_step * m_cloud.width;

  // for (size_t d = 0; d < data.
  m_cloud.data.resize(data.size() * m_cloud.point_step);
  m_cloud.is_bigendian = false;
  m_cloud.is_dense = false;

  for (size_t cp = 0; cp < data.size(); cp++) {
    memcpy(&m_cloud.data[cp * m_cloud.point_step + m_cloud.fields[0].offset],
           &data[cp].x, sizeof(float));
    memcpy(&m_cloud.data[cp * m_cloud.point_step + m_cloud.fields[1].offset],
           &data[cp].y, sizeof(float));
    memcpy(&m_cloud.data[cp * m_cloud.point_step + m_cloud.fields[2].offset],
           &data[cp].z, sizeof(float));

    // for (
  }
  m_pub.publish(m_cloud);
}

void LidarPublisher::PublishPointCloud(std::vector<Position> data) {
  cloud.header.stamp = ros::Time::now();
  cloud.header.seq += 1;

  cloud.points.resize(data.size());
  for (int i = 0; i < data.size(); i++) {
    cloud.points[i].x = data[i].x;
    cloud.points[i].y = data[i].y;
    cloud.points[i].z = data[i].z;
  }

  m_pub_cloud.publish(cloud);
}
