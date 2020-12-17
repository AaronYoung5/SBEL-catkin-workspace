#include "syn_interface/syn_transport_manager.h"

SynTransportManager::SynTransportManager()
    : socket_(*(new boost::asio::io_service)) {}

void SynTransportManager::Connect(std::string hostname, std::string port) {
  int attempt = 0;
  int max_attempts = 5;
  while (true) {
    boost::asio::ip::tcp::resolver resolver(socket_.get_io_service());
    boost::asio::ip::tcp::resolver::query query(hostname, port);
    boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
    boost::asio::ip::tcp::resolver::iterator end;

    boost::system::error_code error = boost::asio::error::host_not_found;
    while (error && iter != end) {
      ROS_DEBUG_STREAM("Trying to connect to " << iter->endpoint() << "...");

      socket_.close();
      socket_.connect(*iter++, error);
    }
    if (error) {
      ROS_WARN_STREAM("Attempt " << attempt << " of " << max_attempts
                                 << " failed.");
      if (++attempt > 5) {
        ROS_FATAL("Connection not established with SynChrono.");
        ROS_FATAL("Shutting Down.");
        ros::shutdown();
        exit(-1);
      }
      ROS_WARN("Trying Again.");
    } else {
      break;
    }
    sleep(1);
  }

  ROS_INFO("Connection Established.");
}

void SynTransportManager::Receive(SynFlatBuffersManager &flatbuffers_manager) {
  // Allocate space for message "header"
  flatbuffers_manager.Buffer().resize(4);
  // Receive just the "header"
  boost::asio::read(
      socket_,
      boost::asio::buffer(flatbuffers_manager.Buffer().data(), (size_t)4));
  // Check the size of the message to read
  int available = ((int *)flatbuffers_manager.Buffer().data())[0];
  // std::cout << "Available :: " << available << std::endl;
  // Allocate space for the message
  flatbuffers_manager.Buffer().resize(available);
  // Receive and record size of received packet
  // Read allows us to read tcp buffer until all (int)available are received
  int received = boost::asio::read(
      socket_,
      boost::asio::buffer(flatbuffers_manager.Buffer().data(), available));
}

void SynTransportManager::Send(SynFlatBuffersManager &flatbuffers_manager) {
  flatbuffers_manager.Finish();
  int32_t size = flatbuffers_manager.GetSize();
  uint8_t *buf = flatbuffers_manager.GetBufferPointer();
  size_t send_size = socket_.send(boost::asio::buffer(buf, size));
}