#pragma once

// ROS includes
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"

// Library includes
#include <boost/asio.hpp>
#include <cmath>
#include <iostream>

// Package includes
#include "ProtobufMessages.pb.h"
#include "chrono_ros_interface/MessageCodes.h"
#include "chrono_ros_interface/SensorStructs.h"

using boost::asio::ip::udp;

class MessageHandler {
private:
  const char *m_port_num;
  udp::socket m_socket;
  boost::asio::ip::udp::endpoint m_simEndpoint;

  std::vector<Vector3D> m_lidar;
  bool m_lidar_all_received;
  Location m_gps;
  IMU m_imu;
  std::vector<Vector3D> m_blue_cones;
  std::vector<Vector3D> m_yellow_cones;
  double m_time;
  Light m_light;
  float m_light_offset;

  Location m_refPosition = {43.070985, -89.400285, 263.2339};
  const double m_earthRadius = 6371000.0;
  const double m_radTodeg = 180.0 / M_PI;

  double m_throttle = 0;
  double m_steering = 0;
  double m_braking = 0;

  Vector3D m_pos;

  bool m_is_ok;

public:
  MessageHandler(const char *port_num = "8080");
  ~MessageHandler() { m_socket.close(); }

  bool ok() { return m_is_ok; }

  void ReceiveAndHandle();
  void Handle(std::vector<uint8_t> buffer, int receieved);
  void Send();

  std::vector<Vector3D> LidarData() { return m_lidar; }
  bool HasAllLidarData() { return m_lidar_all_received; }

  Location GPSData() { return m_gps; }

  IMU IMUData() { return m_imu; }

  std::vector<Vector3D> BlueCones() { return m_blue_cones; }
  std::vector<Vector3D> YellowCones() { return m_yellow_cones; }

  double Time() { return m_time; }

  Vector3D Position() { return m_pos; }

  void TurnLeft() { m_steering <= -1 ? m_steering = -1.0 : m_steering -= .01; }
  void TurnRight() { m_steering >= 1 ? m_steering = 1.0 : m_steering += .01; }
  void IncreaseThrottle() {
    m_throttle >= 1 ? m_throttle = 1 : m_throttle += .01;
  }
  void DecreaseThrottle() {
    m_throttle <= 0 ? m_throttle = 0 : m_throttle -= .01;
  }

  void keyboardCallback(const std_msgs::Int8::ConstPtr &msg);
  void steeringCallback(const std_msgs::Float32::ConstPtr &msg);

private:
};
