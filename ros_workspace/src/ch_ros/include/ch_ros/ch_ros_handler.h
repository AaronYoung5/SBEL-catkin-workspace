#pragma once

// ROS includes
#include "common_msgs/Control.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

// External library includes
#include <boost/asio.hpp>

// Internal package includes
#include "ch_message_codes.h"
#include "publishers.h"

class ChRosHandler {
private:
  // Private variables

  // -- ROS Parameters --- //
  // UDP if false
  bool use_tcp_;
  // Flatbuffers if false
  bool use_protobuf_;

  // --- Message passing protocol ---//
  std::string port_;
  std::string host_name_;
  // TCP
  boost::asio::ip::tcp::socket tcpsocket_;
  boost::asio::ip::tcp::endpoint tcpendpoint_;
  // UDP
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint endpoint_;

  // --- Sensor communication handlers --- //
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
  // Controls send rate
  float send_rate_;
  // Boolean describing if controls have been updated
  bool controls_updated_;

  // Boolean denoting if Chrono is running
  bool chrono_ok_;

public:
  // Public functions

  // --- Constructor --- //
  ChRosHandler(ros::NodeHandle& n, std::string host_name = "localhost",
               std::string port_num = "8080");

  // --- Destructor --- //
  ~ChRosHandler();

  // --- Synchronous receiving functions --- //
  // Receives info over socket and calls necessary handling functions
  void receiveAndHandle();
  void protobufReceiveAndHandle();
  void flatbufferReceiveAndHandle();

  // --- Asynchronous sending functions --- //
  void protobufSendControls();
  void flatbuffersSendControls();

  // Determines if conrtols should update depending on time elapsed
  bool shouldSend();

  // Is Chrono running?
  bool chrono_ok() { return chrono_ok_; }

private:
  // Private functions

  // --- Message handling functions --- //
  // Allocates message reading to helper methods
  // Protobuf
  void handle(std::vector<uint8_t> buffer, int received);
  // Flatbuffers
  void handle(const RosMessage::message *message, int received);

  // Sets target controls to send
  void setTargetControls(const common_msgs::Control::ConstPtr &msg);

  // Sends control message to Chrono
  void sendControls();

  // Initializes the parameters from ROS
  void initializeROSParameters(ros::NodeHandle& n);

  // Initializes and connects to tcp socket
  void initializeSocket();
};
