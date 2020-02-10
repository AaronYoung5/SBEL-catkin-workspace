#include "synchrono_ros/transport/SynROSTransportManager.h"

namespace synchrono {
namespace interface {
SynROSTransportManager::SynROSTransportManager() : SynTransportManager() {}

void SynROSTransportManager::Connect(std::string hostname, std::string port) {
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

} // namespace interface
} // namespace synchrono
