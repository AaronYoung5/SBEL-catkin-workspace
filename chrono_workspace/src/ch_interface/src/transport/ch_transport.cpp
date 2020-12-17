#include "ch_interface/transport/ch_transport.h"

namespace chrono {
namespace transport {
ChTransport::ChTransport(TransportType type, std::string id, int freq)
<<<<<<< HEAD
    : type_(type), id_(id), freq_(freq) {}
=======
    : type_(type), id_(id), freq_(freq), time_(0), num_updates_(0) {}
>>>>>>> d84456083cd1453b3a85c92286e5fa9be41093f3

bool ChTransport::operator==(const std::string &str) {
  return id_.compare(str) == 0;
}

<<<<<<< HEAD
void ChTransport::spinOnce(const ChInterfaceMessage::Message *message) {
} // namespace transport
} // namespace chrono
} // namespace chrono
=======
void ChTransport::spinOnce(double step,
                           ChFlatbufferHandler &flatbuffer_handler) {
  time_ += step;

  if (time_ * freq_ < num_updates_) {
    spinOnce(flatbuffer_handler);
    num_updates_++;
  }
}

} // namespace transport
} // namespace chrono
>>>>>>> d84456083cd1453b3a85c92286e5fa9be41093f3
