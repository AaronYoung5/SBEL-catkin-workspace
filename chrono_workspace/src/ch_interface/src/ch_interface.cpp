#include "ch_interface/ch_interface.h"

namespace chrono {

ChInterface::ChInterface(ros::NodeHandle &n) : transport_manager_(n) {}

void run() {
  while (ros::ok()) {
    ros::spinOnce();
  }
}
} // namespace chrono
