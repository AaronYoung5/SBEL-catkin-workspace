#include <ros/ros.h>

#include "chrono/transport/ch_transport_manager.h"

namespace chrono {
class ChInterface {
private:
  ChTransportManager transport_manager_;

public:
  ChInterface(ros::NodeHandle &n);

  ~ChInterface();

  void run();
}
} // namespace chrono
