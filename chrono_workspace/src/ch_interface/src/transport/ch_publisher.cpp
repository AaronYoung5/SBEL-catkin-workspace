#include "ch_interface/transport/ch_publisher.h"

namespace chrono {
namespace transport {

ChPublisher::ChPublisher(ros::Publisher &pub, TransportType type,
                         std::string id, int freq)
    : ChTransport(type, id, freq), pub_(pub) {}

void ChPublisher::spinOnce(ChFlatbufferHandler &flatbuffer_handler) {}
} // namespace transport
} // namespace chrono
