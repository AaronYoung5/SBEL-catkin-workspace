#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <iostream>

using boost::asio::ip::udp;

class UDPClient {
private:
  // Private class members
  asio::io_service m_io_service;
  udp::socket m_socket;
  udp::endpoint m_endpoint;

public:
  // Public methods
  UDPClient(boost::asio::io_service &io_service, const std::string &host,
            const std::string &port);
  ~UDPClient();
  void send(const std::string& msg);
};
