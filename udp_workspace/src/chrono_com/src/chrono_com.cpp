#include "ros/ros.h"
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::io_context;
using boost::asio::ip::udp;

int main(int argc, char **argv) {
  ros::init(argc,argv,"chrono_com");

  try {
    io_context io_context;

    udp::resolver resolver(io_context);
    udp::endpoint reciever_endpoint = *resolver.resolve(udp::v4(), 8080, "daytime").begin();

    udp::socket socket(io_context);
    socket.open(udp::v4());

    boost::array<char, 1> send_buf = {{ 0 }};
    socket.send_to(boost::asio::buffer(send_buf), reciever_endpoint);

    boost::array<char, 128> recv_buf;
    udp::endpoint sender_endpoint;
  }

  ros::shutdown();
}
