#include "ch_interface/ch_interface.h"

namespace chrono {

ChInterface::ChInterface(ros::NodeHandle &n) : transport_manager_(n) {}

ChInterface::~ChInterface() {}

void ChInterface::run() {
  if (ros::ok()) {
    transport_manager_.startTransport();
    ros::spin();
  }
}
} // namespace chrono
