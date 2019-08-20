#include "ch_interface/transport/ch_publisher.h"

namespace chrono {
namespace transport {

ChPublisher::ChPublisher(ros::Publisher &pub, TransportType type,
                         std::string id, int freq)
    : ChTransport(type, id, freq), pub_(pub) {}

void ChPublisher::update(const ChInterfaceMessage::Message *message) {
  message_ = message;
}

void ChPublisher::spinOnce() {
  switch (type_) {
  case TransportType::CAMERA: {
    auto msg = toImage(
        static_cast<const ChInterfaceMessage::Camera *>(message_->message()));
    pub_.publish(msg);
    break;
  }
  case TransportType::TIME: {
    auto msg = toTime(
        static_cast<const ChInterfaceMessage::Time *>(message_->message()));
    pub_.publish(msg);
    break;
  }
  }
}
} // namespace transport
} // namespace chrono
