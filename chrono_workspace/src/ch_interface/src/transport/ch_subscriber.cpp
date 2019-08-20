#include "ch_interface/transport/ch_subscriber.h"

namespace chrono {
namespace transport {
ChSubscriber::ChSubscriber(ros::Subscriber &sub, TransportType type,
                           std::string id, int freq)
    : ChTransport(type, id, freq), sub_(sub) {}

void ChSubscriber::spinOnce() {}
} // namespace transport
} // namespace chrono
