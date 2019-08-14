#include <ros/ros.h>
#include <boost/asio.hpp>
#include <thread>
#include <mutex>

namespace chrono {
namespace transport {
class ChTransportManager {
private:
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;

  ChTransport std::vector<ChTransport> transports_;

  std::thread thread_;
  std::mutex mutex_;

public:
  ChTransportManager(ros::NodeHandle &n);
  ~ChTransportManager();

  void spinOnce();

private:
  loadParameters(ros::NodeHandle &n);
  initChrono();
  handleSensorData(const boost::system::error_code &err,
                   std::size_t bytes_transferred);
};
} // namespace transport
} // namespace chrono
