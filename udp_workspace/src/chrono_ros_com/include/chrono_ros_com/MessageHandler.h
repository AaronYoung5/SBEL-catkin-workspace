#pragma once

#include "ProtobufMessages.pb.h"
#include "chrono_ros_com/MessageCodes.h"
#include <boost/asio.hpp>
#include <iostream>
#include <cmath>

using boost::asio::ip::udp;

struct Quaternion {
  double x;
  double y;
  double z;
  double w;
};

struct Position {
  double x;
  double y;
  double z;

  Position(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  Position() {}
};

struct GPSPosition {
  double lat;
  double lon;
  double alt;
};

class MessageHandler {
private:
  const char *m_port_num;
  udp::socket m_socket;
  boost::asio::ip::udp::endpoint m_simEndpoint;

  std::vector<Position> m_lidarData;
  Position m_position;
  Quaternion m_orientation;
  double m_yaw;
  double m_time;
  double m_lightXPos, m_lightYPos;
  int m_lightXDir, m_lightYDir;
  double m_lightOffset;

  GPSPosition m_refPosition = { 43.070985, -89.400285, 263.2339 };
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

  Position Pose() { return m_position; }
  Quaternion Orientation() { return m_orientation; }
  double Time() { return m_time; }

  void TurnLeft() { m_steering <= -1 ? m_steering=-1.0 : m_steering-=.01; }
  void TurnRight() { m_steering >= 1 ? m_steering=1.0 : m_steering+=.01; }
  void IncreaseThrottle() { m_throttle >= 1 ? m_throttle=1 : m_throttle+=.01; }
  void DecreaseThrottle() { m_throttle <= 0 ? m_throttle=0 : m_throttle-=.01; }

private:
  Position ToRelativePosition(DriverMessages::gps message);
  Quaternion ToQuaternion(DriverMessages::imu message);
};
