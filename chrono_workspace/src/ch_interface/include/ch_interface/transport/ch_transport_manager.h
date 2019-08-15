#pragma once

#include <boost/asio.hpp>
#include <mutex>
#include <ros/ros.h>
#include <thread>

#include "ch_interface/flatbuffer/ch_flatbuffer_converter.h"

#include "ch_publisher.h"
#include "ch_subscriber.h"
#include "ch_transport.h"
#include "ch_transport_type.h"

#include "rosgraph_msgs/Clock.h"

using namespace chrono::flatbuffer;

namespace chrono {
namespace transport {
class ChTransportManager
    : public std::enable_shared_from_this<ChTransportManager> {
private:
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;

  std::vector<ChTransport> transports_;

  std::thread thread_;
  std::mutex mutex_;

  std::string host_name_, port_num_;

  bool closed_;
  std::vector<uint8_t> buffer_;

public:
  ChTransportManager(ros::NodeHandle &n);
  ~ChTransportManager();

  void startTransport();

private:
  void loadParameters(ros::NodeHandle &n);
  void initChrono();
  void readTransportMessage();
  void handleTransportMessage(size_t size);
};
} // namespace transport
} // namespace chrono
