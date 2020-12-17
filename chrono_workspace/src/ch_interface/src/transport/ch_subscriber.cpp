#include "ch_interface/transport/ch_subscriber.h"

namespace chrono {
namespace transport {
<<<<<<< HEAD
ChSubscriber::ChSubscriber(std::shared_ptr<boost::asio::ip::tcp::socket> socket,
                           TransportType type, int num,
                           ros::Subscriber &sub, int update_rate)
    : ChTransport(socket,type,num), sub_(sub) {}

void ChSubscriber::spinOnce() {}
=======
ChSubscriber::ChSubscriber(ros::Subscriber &sub, TransportType type,
                           std::string id, int freq)
    : ChTransport(type, id, freq), sub_(sub) {}

void ChSubscriber::spinOnce(ChFlatbufferHandler &flatbuffer_handler) {}
>>>>>>> d84456083cd1453b3a85c92286e5fa9be41093f3
} // namespace transport
} // namespace chrono
