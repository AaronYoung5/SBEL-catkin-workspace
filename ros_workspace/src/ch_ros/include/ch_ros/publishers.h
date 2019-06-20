#pragma once

// Ros includes
#include "common_msgs/ConeMap.h"
#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"

// External package includes
// #include <common_utilities/Vector.h>

// Internal package includes
#include "protobuf_messages.pb.h"

template <typename msgtype> class Publisher {
protected:
  msgtype data_;
  ros::Publisher pub_;

public:
  Publisher(ros::NodeHandle n, std::string node_name, int queue_size) {
    pub_ = n.advertise<msgtype>(node_name, queue_size);
  }

  virtual void publish(std::vector<uint8_t> buffer, int received) = 0;
};

// --------------------------------- Lidar ---------------------------------- //
class Lidar : Publisher<sensor_msgs::PointCloud2> {
private:
  int expected_, last_id_ = 0;
  bool is_complete_;
public:
  Lidar(ros::NodeHandle n, std::string node_name, int queue_size);
  void publish(std::vector<uint8_t> buffer, int received);
  void tcppublish(std::vector<uint8_t> buffer, int received);
};

// --------------------------------- IMU ---------------------------------- //
class IMU : Publisher<sensor_msgs::Imu> {
public:
  IMU(ros::NodeHandle n, std::string node_name, int queue_size);
  void publish(std::vector<uint8_t> buffer, int received);
};

// --------------------------------- GPS ---------------------------------- //
class GPS : Publisher<sensor_msgs::NavSatFix> {
public:
  GPS(ros::NodeHandle n, std::string node_name, int queue_size);
  void publish(std::vector<uint8_t> buffer, int received);
};

// --------------------------------- TIME ---------------------------------- //
class Time : Publisher<rosgraph_msgs::Clock> {
public:
  Time(ros::NodeHandle n, std::string node_name, int queue_size);
  void publish(std::vector<uint8_t> buffer, int received);
};

// --------------------------------- Cones ---------------------------------- //
class Cones : Publisher<common_msgs::ConeMap> {
public:
  Cones(ros::NodeHandle n, std::string node_name, int queue_size);
  void publish(std::vector<uint8_t> buffer, int received);
};
