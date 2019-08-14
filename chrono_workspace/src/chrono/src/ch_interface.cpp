#include "chrono/ch_interface.h"

namespace chrono {

ChInterface::ChInterface(ros::NodeHandle &n) : transport_manager_(n) {}

void run() {
  while (ros::ok() && transport_manager_.ok()) {
    transport_manager.spinOnce();
    ros::spinOnce();
  }
}
} // namespace chrono
