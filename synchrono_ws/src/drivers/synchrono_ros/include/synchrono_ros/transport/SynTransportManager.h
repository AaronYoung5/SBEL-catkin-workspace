#pragma once

#include <ros/ros.h>

#include <boost/asio.hpp>

#include "synchrono_ros/flatbuffers/SynFlatBuffersManager.h"

#include "synchrono_ros/flatbuffers/messages/SynPointerMessage.h"

class SynTransportManager {
private:
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;

  SynFlatBuffersManager &flatbuffers_manager_;

public:
  SynTransportManager(SynFlatBuffersManager &flatbuffers_manager);
  ~SynTransportManager();
  void Connect(std::string hostname, std::string port);

  void Send();
  void Receive();
};
