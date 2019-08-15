#pragma once
#define FLATBUFFERS_DEBUG_VERIFICATION_FAILURE
#define FLATBUFFERS_ASSERT
// Ros includes
#include "common_msgs/ConeMap.h"
#include "common_msgs/VehState.h"
#include "common_srvs/ConeMap.h"
#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/image_encodings.h>

#include "image_transport/image_transport.h"

// External package includes
// #include <common_utilities/Vector.h>

// Internal package includes
#include "ch_ros/ChRosMessages_generated.h"

template <typename msgtype> class Publisher {
protected:
  msgtype data_;
  ros::Publisher pub_;
  bool data_received_ = false;

public:
  Publisher(ros::NodeHandle n, std::string node_name, int queue_size) {
    if (queue_size != 0)
      pub_ = n.advertise<msgtype>(node_name, queue_size);
  }

  virtual void publish(const ChRosMessage::Message *message, int received) = 0;
};

// --------------------------------- Camera ----------------------------------
// //
class Camera : Publisher<sensor_msgs::Image> {
private:
  image_transport::Publisher image_pub_;

public:
  Camera(ros::NodeHandle n, std::string node_name, int queue_size);
  void publish(const ChRosMessage::Message *message, int received);
};

// --------------------------------- Lidar ---------------------------------- //
class Lidar : Publisher<sensor_msgs::PointCloud2> {
public:
  Lidar(ros::NodeHandle n, std::string node_name, int queue_size);
  void publish(const ChRosMessage::Message *message, int received);
};

// --------------------------------- IMU ---------------------------------- //
class IMU : Publisher<sensor_msgs::Imu> {
public:
  IMU(ros::NodeHandle n, std::string node_name, int queue_size);
  void publish(const ChRosMessage::Message *message, int received);
};

// --------------------------------- GPS ---------------------------------- //
class GPS : Publisher<sensor_msgs::NavSatFix> {
public:
  GPS(ros::NodeHandle n, std::string node_name, int queue_size);
  void publish(const ChRosMessage::Message *message, int received);
};

// --------------------------------- TIME ---------------------------------- //
class Time : Publisher<rosgraph_msgs::Clock> {
private:
  float time_;

public:
  Time(ros::NodeHandle n, std::string node_name, int queue_size);
  void publish(const ChRosMessage::Message *message, int received);

  float GetTime() { return time_; }
};

// --------------------------------- CONES ---------------------------------- //
class Cones : Publisher<common_msgs::ConeMap> {
private:
  ros::ServiceServer srv_;

public:
  Cones(ros::NodeHandle n, std::string node_name, int queue_size);
  void publish(const ChRosMessage::Message *message, int received);
  bool send_cones(common_srvs::ConeMap::Request &req,
                  common_srvs::ConeMap::Response &res);
};

// --------------------------------- VEHICLE ----------------------------------
// //
class Vehicle : Publisher<common_msgs::VehState> {
public:
  Vehicle(ros::NodeHandle n, std::string node_name, int queue_size);
  void publish(const ChRosMessage::Message *message, int received);
};
