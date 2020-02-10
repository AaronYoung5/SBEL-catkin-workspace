#pragma once

#include <ros/ros.h>

#include <boost/asio.hpp>

#include "synchrono_interface/transport/SynTransportManager.h"
#include "synchrono_ros/SynApi.h"

namespace synchrono {
namespace interface {
class SYN_API SynROSTransportManager : public SynTransportManager {
public:
  SynROSTransportManager();
  // ~SynROSTransportManager();

  virtual void Connect(std::string hostname, std::string port);
};
} // namespace interface
} // namespace synchrono
