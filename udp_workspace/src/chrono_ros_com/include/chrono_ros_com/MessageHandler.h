#pragma once

#include "ProtobufMessages.pb.h"
#include "chrono_ros_com/MessageCodes.h"
#include <boost/asio.hpp>
#include <cmath>
#include <iostream>

using boost::asio::ip::udp;

struct Quaternion {
  double x;
  double y;
  double z;
  double w;
};

struct Position {
  float x;
  float y;
  float z;

  Position(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
  Position() {}
};

struct GPSPosition {
  double lat;
  double lon;
  double alt;
};

struct IMU {
  struct Vector {
    float x;
    float y;
    float z;
  };
  Vector linear_acceleration;
  Vector angular_velocity;
};

class MessageHandler {
private:
  const char *m_port_num;
  udp::socket m_socket;
  boost::asio::ip::udp::endpoint m_simEndpoint;

  std::vector<Position> m_lidarData;
  GPSPosition m_gpsPose;
  IMU m_imu;
  Position m_position;
  std::vector<Position> m_blue_cones;
  std::vector<Position> m_yellow_cones;
  Quaternion m_orientation;
  double m_yaw;
  double m_time;
  double m_lightXPos, m_lightYPos;
  int m_lightXDir, m_lightYDir;
  double m_lightOffset;

  GPSPosition m_refPosition = {43.070985, -89.400285, 263.2339};
  const double m_earthRadius = 6371000.0;
  const double m_radTodeg = 180.0 / M_PI;

  double m_throttle = 0;
  double m_steering = 0;
  double m_braking = 0;

public:
  MessageHandler(const char *port_num = "8080");
  ~MessageHandler() { m_socket.close(); }

  void ReceiveAndHandle();
  void Handle(std::vector<uint8_t> buffer, int receieved);
  void Send();

  std::vector<Position> LidarData() { return m_lidarData; }
  GPSPosition GPSData() { return m_gpsPose; }
  IMU IMUData() { return m_imu; }
  std::vector<Position> BlueCones() { return m_blue_cones; }
  std::vector<Position> YellowCones() { return m_yellow_cones; }

  Position Pose() { return m_position; }
  Quaternion Orientation() { return m_orientation; }
  double Time() { return m_time; }

  void TurnLeft() { m_steering <= -1 ? m_steering = -1.0 : m_steering -= .01; }
  void TurnRight() { m_steering >= 1 ? m_steering = 1.0 : m_steering += .01; }
  void IncreaseThrottle() {
    m_throttle >= 1 ? m_throttle = 1 : m_throttle += .01;
  }
  void DecreaseThrottle() {
    m_throttle <= 0 ? m_throttle = 0 : m_throttle -= .01;
  }

private:
  Position ToRelativePosition(DriverMessages::gps message);
  Quaternion ToQuaternion(DriverMessages::imu message);
};
