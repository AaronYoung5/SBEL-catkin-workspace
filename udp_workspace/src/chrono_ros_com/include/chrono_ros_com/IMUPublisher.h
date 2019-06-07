#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "chrono_ros_com/MessageHandler.h"

class IMUPublisher {
private:
  MessageHandler &m_handler;

  ros::NodeHandle m_n;

  ros::Publisher m_pub;

  sensor_msgs::Imu m_imu;

public:
  IMUPublisher(ros::NodeHandle n, MessageHandler &handler);

  void PublishIMU();
};
