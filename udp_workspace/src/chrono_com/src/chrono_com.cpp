#include "ros/ros.h"
#include <boost/asio.hpp>

using boost::asio::ip::udp;

int main(int argc, char **argv) {
  ros::init(argc, argv, "chrono_com");

  const char* portNum = "8080";

  boost::asio::ip::udp::socket socket(
      *(new boost::asio::io_service),
      boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                                     std::atoi(portNum)));

  ros::shutdown();
}
