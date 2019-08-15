#include "ch_interface/transport/ch_publisher.h"

namespace chrono {
namespace transport {

ChPublisher::ChPublisher(ros::Publisher &pub, std::string id, int freq)
    : ChTransport(id, freq), pub_(pub) {}

template <class msg_type>
void ChPublisher::operator()(TransportType type,
                             const ChInterfaceMessage::Message *message) {
  auto msg = fb_converter_.convert<msg_type>(type, message);
  pub_.publish(msg);
}
} // namespace transport
} // namespace chrono
