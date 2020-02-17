#include "synchrono_ros/transport/SynTransportManager.h"

SynTransportManager::SynTransportManager(
    SynFlatBuffersManager &flatbuffers_manager)
    : flatbuffers_manager_(flatbuffers_manager),
      socket_(std::make_shared<boost::asio::ip::tcp::socket>(
          *new boost::asio::io_service)) {}

void SynTransportManager::Connect(std::string hostname, std::string port) {
  int attempt = 0;
  int max_attempts = 5;
  while (true) {
    boost::asio::ip::tcp::resolver resolver(socket_->get_io_service());
    boost::asio::ip::tcp::resolver::query query(hostname, port);
    boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
    boost::asio::ip::tcp::resolver::iterator end;

    boost::system::error_code error = boost::asio::error::host_not_found;
    while (error && iter != end) {
      ROS_DEBUG_STREAM("Trying to connect to " << iter->endpoint() << "...");

      socket_->close();
      socket_->connect(*iter++, error);
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

SynTransportManager::~SynTransportManager() {
  socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
  socket_->close();
}

void SynTransportManager::Send() {
  flatbuffers_manager_.FinishSizePrefixed();
  int32_t size = flatbuffers_manager_.GetSize();
  uint8_t *buf = flatbuffers_manager_.GetBufferPointer();
  size_t send_size = socket_->send(boost::asio::buffer(buf, size));
  std::cout << "Send Size:: " << send_size << std::endl;
}

void SynTransportManager::Receive() {
  // Allocate space for message "header"
  std::vector<uint8_t> buffer(4);
  // Receive just the "header"
  boost::asio::read(*socket_, boost::asio::buffer(buffer.data(), 4));
  // Check the size of the message to read
  int available = ((int *)buffer.data())[0];
  // std::cout << "Available :: " << available << std::endl;
  // Allocate space for the message
  buffer.resize(available);
  // Receive and record size of received packet
  // Read allows us to read tcp buffer until all (int)available are received
  size_t received = boost::asio::read(
      *socket_, boost::asio::buffer(buffer.data(), available));
  std::cout << "Received :: " << received << std::endl;
  SynPointerMessage *msg = new SynPointerMessage(0, buffer.data(), received);
  flatbuffers_manager_.AddMessage(msg);
  delete msg;
  flatbuffers_manager_.GetBuffer() = buffer;
  // flatbuffers_manager_.Finish();
}
