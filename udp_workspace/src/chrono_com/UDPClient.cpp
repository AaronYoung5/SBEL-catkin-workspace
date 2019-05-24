#include "chrono_com/UDPClient.h"

UDPClient::UDPClient(boost::asio::io_service &io_service,
                     const std::string &host, const std::string &port)
    : m_io_service(io_service),
      m_socket(io_service, udp::endpoint(udp::v4(), 0)) {
  udp::resolver resolver(m_io_service);
  udp::resolver::query query(udp::v4(), host, port);
  udp::resolver::iterator iter = resolver.resolve(query);
  endpoint_ = *iter;
}

UDPClient::~UDPClient() { m_socket.close(); }

UDPClient::send(const std::string &msg) {
  m_socket.send_to(boost::asio::buffer(msg, msg.size()), m_endpoint);
}
