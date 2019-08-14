#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <mutex>
#include <thread>

namespace chrono {
namespace transport {

enum TransportType { CAMERA, LIDAR, GPS, IMU, CONTROL, TIME, EXIT, CONFIG };

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

class ChTransport {
public:
  ChTransport(std::shared_ptr<boost::asio::ip::tcp::socket> socket);

  virtual void Advance() = 0;
};

class ChSender : public ChTransport {
private:

public:
  ChSender(std::shared_ptr<boost::asio::ip::tcp::socket> socket,
              ros::NodeHandle &n, std::string topic_name, int queue_size,
              int update_rate);
};

class ChReceiver : public ChTransport {
private:

public:
  ChReceiver(std::shared_ptr<boost::asio::ip::tcp::socket> socket,
              ros::NodeHandle &n, std::string topic_name, int queue_size,
              int update_rate);
};

class ChPublisher : public ChTransport {
private:
  ros::Publisher pub_;

public:
  ChPublisher(std::shared_ptr<boost::asio::ip::tcp::socket> socket,
              ros::NodeHandle &n, std::string topic_name, int queue_size,
              int update_rate);

  void Advance();
};

class ChSubscriber : public ChTransport {
private:
  ros::Subscriber sub_;

public:
  ChSubscriber(std::shared_ptr<boost::asio::ip::tcp::socket> socket,
               ros::NodeHandle &n, std::string topic_name, int queue_size,
               int update_rate);

  void Advance();
};
} // namespace transport
} // namespace chrono
