// #pragma once
//
// #include <ros/ros.h>
//
// #include "ch_transport.h"
//
// namespace chrono {
// namespace transport {
// class ChSubscriber : public ChTransport {
// private:
//   ros::Subscriber sub_;
//
// public:
//   ChSubscriber(std::shared_ptr<boost::asio::ip::tcp::socket> socket,
//                TransportType type, int num, ros::Subscriber &sub,
//                int update_rate);
//
//   void spinOnce();
// };
// } // namespace transport
// } // namespace chrono
