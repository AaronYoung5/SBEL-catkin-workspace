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

class FlatBufferConverter {
protected:
  ros::Publisher pub_;
  image_transport::Publisher image_pub_;

public:
  FlatBufferConverter(ros::Publisher pub) : pub_(pub) {}
  FlatBufferConverter(image_transport::Publisher pub) : image_pub_(pub) {}

  virtual void publish(const ChRosMessage::Message *message, int received) = 0;
};

// --------------------------------- Cam ---------------------------------- //
class Camera : FlatBufferConverter {
public:
  Camera(ros::NodeHandle &n, std::string node_name, int queue_size);
  void ConvertAndPublish(const ChRosMessage::Message *message, int received);
};

// --------------------------------- Lidar ---------------------------------- //
class Lidar : FlatBufferConverter {
public:
  Lidar(ros::NodeHandle &n, std::string node_name, int queue_size);
  void publish(const ChRosMessage::Message *message, int received);
};

// --------------------------------- IMU ---------------------------------- //
class IMU : FlatBufferConverter {
public:
  IMU(ros::NodeHandle &n, std::string node_name, int queue_size);
  void publish(const ChRosMessage::Message *message, int received);
};

// --------------------------------- GPS ---------------------------------- //
class GPS : FlatBufferConverter {
public:
  GPS(ros::NodeHandle &n, std::string node_name, int queue_size);
  void publish(const ChRosMessage::Message *message, int received);
};

// --------------------------------- TIME ---------------------------------- //
class Time : FlatBufferConverter {
private:
  float time_;

public:
  Time(ros::NodeHandle &n, std::string node_name, int queue_size);
  void publish(const ChRosMessage::Message *message, int received);

  float GetTime() { return time_; }
};

// --------------------------------- CONES ---------------------------------- //
class Cones : FlatBufferConverter {
private:
  ros::ServiceServer srv_;

public:
  Cones(ros::NodeHandle &n, std::string node_name, int queue_size);
  void publish(const ChRosMessage::Message *message, int received);
  bool send_cones(common_srvs::ConeMap::Request &req,
                  common_srvs::ConeMap::Response &res);
};

// --------------------------------- VEHICLE ----------------------------------
// //
class Vehicle : FlatBufferConverter {
public:
  Vehicle(ros::NodeHandle &n, std::string node_name, int queue_size);
  void publish(const ChRosMessage::Message *message, int received);
};
