#pragma once

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "chrono_ros_com/MessageHandler.h"

class GPSPublisher {
private:
  MessageHandler &m_handler;

  ros::NodeHandle m_n;

  ros::Publisher m_pub;

  sensor_msgs::NavSatFix m_gps;

public:
  GPSPublisher(ros::NodeHandle n, MessageHandler &handler);

  void PublishGPS();
};
