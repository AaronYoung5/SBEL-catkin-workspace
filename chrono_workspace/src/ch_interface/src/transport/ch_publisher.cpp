#include "ch_interface/transport/ch_publisher.h"

namespace chrono {
namespace transport {

ChPublisher::ChPublisher(ros::Publisher &pub, TransportType type,
                         std::string id, int freq)
    : ChTransport(type, id, freq), pub_(pub) {}

<<<<<<< HEAD
void ChPublisher::spinOnce(const ChInterfaceMessage::Message *message) {
  switch (type_) {
  case TransportType::CAMERA: {
    pub_.publish(toImage(
        static_cast<const ChInterfaceMessage::Camera *>(message->message())));
    break;
  }
  case TransportType::TIME: {
    pub_.publish(toTime(
        static_cast<const ChInterfaceMessage::Time *>(message->message())));
    break;
  }
  }
}
=======
void ChPublisher::spinOnce(ChFlatbufferHandler &flatbuffer_handler) {}
>>>>>>> d84456083cd1453b3a85c92286e5fa9be41093f3
} // namespace transport
} // namespace chrono
