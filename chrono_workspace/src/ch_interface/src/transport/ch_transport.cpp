#include "ch_interface/transport/ch_transport.h"

namespace chrono {
namespace transport {
ChTransport::ChTransport(TransportType type, std::string id, int freq)
    : type_(type), id_(id), freq_(freq) {}

bool ChTransport::operator==(const std::string &str) {
  return id_.compare(str) == 0;
}

// void ChTransport::spinOnce(const ChInterfaceMessage::Message *message) {}
} // namespace transport
} // namespace chrono
