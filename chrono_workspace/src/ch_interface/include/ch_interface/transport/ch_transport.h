#pragma once

#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <mutex>
#include <thread>

namespace chrono {
namespace transport {

class ChTransport {
private:
  std::string id_;
  int freq_;

public:
  ChTransport(std::string id, int freq);

  // bool operator!=()

  std::string id() const { return id_; }

  virtual void spinOnce();
};
} // namespace transport
} // namespace chrono
