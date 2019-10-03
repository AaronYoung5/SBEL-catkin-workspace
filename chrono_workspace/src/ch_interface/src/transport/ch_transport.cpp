#include "ch_interface/transport/ch_transport.h"

namespace chrono {
namespace transport {
ChTransport::ChTransport(TransportType type, std::string id, int freq)
    : type_(type), id_(id), freq_(freq), time_(0), num_updates_(0) {}

bool ChTransport::operator==(const std::string &str) {
  return id_.compare(str) == 0;
}

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
