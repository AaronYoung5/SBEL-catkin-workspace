#pragma once

#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <mutex>
#include <thread>

#include "ch_interface/flatbuffer/ch_flatbuffer_handler.h"
#include "ch_transport_type.h"

using namespace chrono::flatbuffer;

namespace chrono {
namespace transport {

class ChTransport {
protected:
  TransportType type_;
  std::string id_;
  int freq_;
  double time_;
  int num_updates_;

public:
  ChTransport(TransportType type, std::string id, int freq);

  bool operator==(const std::string &str);

  TransportType type() const { return type_; }
  std::string id() const { return id_; }
  int freq() const { return freq_; }

  virtual void spinOnce(double step, ChFlatbufferHandler& flatbuffer_handler);
  virtual void spinOnce(ChFlatbufferHandler& flatbuffer_handler) = 0;
};
} // namespace transport
} // namespace chrono
