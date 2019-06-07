#pragma once

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "chrono_ros_com/MessageHandler.h"

class ConePublisher {
private:
  MessageHandler &m_handler;

  ros::NodeHandle m_n;

  ros::Publisher m_pub;

  geometry_msgs::PoseArray m_cones;

public:
  ConePublisher(ros::NodeHandle n, MessageHandler &handler);

  void PublishCones();
};
