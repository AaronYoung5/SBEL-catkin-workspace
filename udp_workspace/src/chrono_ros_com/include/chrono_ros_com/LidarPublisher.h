#pragma once

#include "ros/ros.h"
#include "MessageHandler.h"

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"

class LidarPublisher {
private:
  MessageHandler &m_handler;

  ros::NodeHandle n;

  ros::Publisher m_pub;
  ros::Publisher m_pub_cloud;

  sensor_msgs::PointCloud2 m_cloud;
  sensor_msgs::PointCloud cloud;
public:
  LidarPublisher(int argc, char **argv, MessageHandler& handler);

  void Publish(std::vector<Position> data);
  void PublishPointCloud(std::vector<Position> data);
};
