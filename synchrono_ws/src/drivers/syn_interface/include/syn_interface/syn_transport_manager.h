#pragma once

#include <ros/ros.h>

#include <boost/asio.hpp>
#include <mutex>

#include "syn_interface/syn_flatbuffers_manager.h"

class SynTransportManager {
private:
  boost::asio::ip::tcp::socket socket_;

  std::vector<uint8_t> buffer_;

public:
  SynTransportManager();
  void Connect(std::string hostname, std::string port);

  void Receive(SynFlatBuffersManager& flatbuffers_manager);
  void Send(SynFlatBuffersManager& flatbuffers_manager);
};
