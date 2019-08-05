#pragma once

// ROS include
#include "ros/ros.h"

// ROS message includes
#include "common_msgs/Control.h"
#include "sensor_msgs/PointCloud2.h"

// External library includes
#include <boost/asio.hpp>

// Internal package includes
#include "publishers.h"

class ChRosHandler {
private:
  // Private variables

  // -- ROS Parameters --- //
  // Don't visualize if false
  bool use_irrlicht_;
  // Socket host name and port number
  std::string host_name_;
  std::string port_num_;

  // --- Message passing protocol ---//
  boost::asio::ip::tcp::socket socket_;

  // --- Sensor communication handlers --- //
  Camera camera_;
  Lidar lidar_;
  IMU imu_;
  GPS gps_;
  Time time_;
  Cones cones_;
  Vehicle vehicle_;

  // --- Controlls --- //
  // Subscriber
  ros::Subscriber control_sub_;
  // Respective vehicle values
  float throttle_, steering_, braking_;

  // Boolean denoting if Chrono is running
  bool chrono_ok_;

public:
  // Public functions

  // --- Constructor --- //
  ChRosHandler(ros::NodeHandle &n);

  // --- Destructor --- //
  ~ChRosHandler();

  // --- Synchronous receiving functions --- //
  // Uses ros param to specify determine correct receive/handle function
  void receive();

  // Is Chrono running?
  bool chrono_ok() { return chrono_ok_; }

private:
  // Private functions

  // --- Message handling functions --- //
  // Allocates message reading to helper methods
  void handle(const ChRosMessage::Message *message, int received);

  // Sets target controls to send
  void setTargetControls(const common_msgs::Control::ConstPtr &msg);

  // --- Sending functions --- //
  // Sends configuration message to Chrono
  void sendConfig();
  // Sends control message to Chrono
  void sendControls();

  // Initializes the parameters from ROS
  void initializeROSParameters(ros::NodeHandle &n);

  // Initializes and connects to tcp socket
  void initializeSocket();
};
