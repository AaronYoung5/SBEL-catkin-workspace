#pragma once

// ROS includes
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

// External library includes
#include <boost/asio.hpp>
// #include <cmath>
// #include <iostream>

// Internal package includes
// #include ""
#include "ch_message_codes.h"
#include "publishers.h"

using boost::asio::ip::udp;

class ChRosHandler {
private:
  // Private variables

  // Message passing protocol
  const char* port_num_;
  udp::socket socket_;
  boost::asio::ip::udp::endpoint endpoint_;

  // Sensors
  Lidar lidar_;
  IMU imu_;
  GPS gps_;
  Time time_;
  Cones cones_;

  // Boolean denoting if Chrono is running
  bool ok_;

public:
  // Public functions

  // Constructor
  ChRosHandler(ros::NodeHandle n, const char *port_num = "8080");

  // Destructor
  // Closes socket
  ~ChRosHandler() { socket_.close(); }

  // Looks for and receives info over the socket and calls necessary handling methods
  void receiveAndHandle();

  // Is Chrono running?
  bool ok() { return ok_; }

private:
  // Private functions

  // Allocates message reading to helper methods
  void handle(std::vector<uint8_t> buffer, int received);

  // Sends control message to Chrono
  void sendControls();
};
