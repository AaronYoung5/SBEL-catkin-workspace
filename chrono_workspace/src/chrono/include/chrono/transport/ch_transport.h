#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <mutex>
#include <thread>

namespace chrono {
namespace transport {

enum TransportType { CAMERA, LIDAR, GPS, IMU, CONTROL, TIME, EXIT, CONFIG };

class ChTransport {
private:
  std::string id_;

public:
  ChTransport(std::shared_ptr<boost::asio::ip::tcp::socket> socket,
              std::string id);

  std::string ID() { return id_; }

  virtual void spinOnce();
};
} // namespace transport
} // namespace chrono
