#pragma once

#include "MessageHandler.h"
#include "ros/ros.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

class LidarPublisher {
private:
  MessageHandler &m_handler;

  ros::NodeHandle m_n;

  ros::Publisher m_pub;
  ros::Publisher m_pub2;

  sensor_msgs::PointCloud m_cloud;
  sensor_msgs::PointCloud2 m_cloud2;

public:
  LidarPublisher(ros::NodeHandle n, MessageHandler &handler);

  void PublishPointCloud2();
  void PublishPointCloud();
  void PublishConvertedPCToPC2();
};
