#include "ch_interface/transport/ch_subscriber.h"

namespace chrono {
namespace transport {
ChSubscriber::ChSubscriber(std::shared_ptr<boost::asio::ip::tcp::socket> socket,
                           TransportType type, int num,
                           ros::Subscriber &sub, int update_rate)
    : ChTransport(socket,type,num), sub_(sub) {}

void ChSubscriber::spinOnce() {}
} // namespace transport
} // namespace chrono
