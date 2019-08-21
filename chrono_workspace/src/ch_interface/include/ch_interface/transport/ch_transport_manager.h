#pragma once

#include <boost/asio.hpp>
#include <mutex>
#include <ros/ros.h>
#include <thread>

#include "ch_interface/flatbuffer/ch_config_message_generated.h"
#include "ch_interface/flatbuffer/ch_flatbuffer_converter.h"
#include "ch_interface/flatbuffer/ch_flatbuffer_handler.h"
#include "ch_interface/flatbuffer/ch_interface_messages_generated.h"

#include "ch_publisher.h"
#include "ch_subscriber.h"
#include "ch_transport.h"
#include "ch_transport_type.h"

#include "rosgraph_msgs/Clock.h"

typedef std::vector<flatbuffers::Offset<ChConfigMessage::Sensor>>
    FlatbufferSensorVector;

using namespace chrono::flatbuffer;

namespace chrono {
namespace transport {
class ChTransportManager {
private:
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  boost::asio::ip::tcp::resolver resolver_;

  ChFlatbufferHandler flatbuffer_handler_;

  std::vector<std::shared_ptr<ChTransport>> transports_;

  std::thread thread_;
  std::mutex mutex_;

  std::string host_name_, port_num_;

public:
  ChTransportManager(ros::NodeHandle &n);
  ~ChTransportManager();

  void startTransport();

private:
  void establishConnection();
  void async_start(int &count);
  void resolve_handler(const boost::system::error_code &ec,
                       boost::asio::ip::tcp::resolver::iterator it, int &count);
  void connect_handler(const boost::system::error_code &ec,
                       boost::asio::ip::tcp::resolver::iterator it, int &count);
  void loadParameters(ros::NodeHandle &n);
  void initChrono(flatbuffers::FlatBufferBuilder &builder,
                  FlatbufferSensorVector &flatbuffer_transports);
  void readTransportMessage();
  void handleTransportMessage(size_t size);
};
} // namespace transport
} // namespace chrono
