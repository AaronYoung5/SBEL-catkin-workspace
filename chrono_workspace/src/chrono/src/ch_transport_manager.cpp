#include "chrono/ch_transport_manager.h"

namespace chrono {
namespace transport {
ChTransportManager::ChTransportManager(ros::NodeHandle &n)
    : socket_(std::make_shared<boost::asio::ip::tcp::socket>(
          *new boost::asio::io_service)) {
  loadParameters(n);
  initChrono();
}

ChTransportManager::~ChTransportManager() {
  socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
  if (thread_.joinable())
    thread_.join();
}

void ChTransportManager::loadParameters(ros::NodeHandle &n) {
  n.param("chrono/use_irrlicht", use_irrlicht_, true);
  n.param<std::string>("chrono/hostname", host_name_, "localhost");
  n.param<std::string>("chrono/port", port_num_, "8080");

  float simulation_step_size;
  n.param("chrono/simulation_step_size", simulation_step_size, "8080");

  transports_.push_back(
      ChPublisher(socket_, n, "clock", "clock", 1, simulation_step_size));

  const char *sensors[] = {"camera", "lidar", "gps", "imu"};

  size_t i = 0;

  for (const char *sensor : sensors) {
    std::stringstream ss;
    ss << sensor << i;
    std::string id = ss.str();
    std::string param_name = std::string("chrono/sensors/") + ss.str();
    if (n.hasParam(param_name)) {
      int sensor_type;
      n.getParam(param_name, sensor_type);

      std::string topic_name;
      n.getParam(param_name + std::string("_topic"), topic_name);

      int freq;
      n.getParam(param_name + std::string("_freq"), freq);

      int queue_size;
      n.getParam(param_name + std::string("_queue_size"), queue_size);

      transports_.push_back(
          ChPublisher(socket_, n, id, topic_name, queue_size, freq));
    }
  }
}

void ChTransportManager::initSocket() {
  // Initialize socket
  int counter = 0;
  int max_attempts = 5;
  while (true) {
    boost::asio::ip::tcp::resolver resolver(socket_->get_io_service());
    boost::asio::ip::tcp::resolver::query query(host_name_, port_num_);
    boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
    boost::asio::ip::tcp::resolver::iterator end;

    boost::system::error_code error = boost::asio::error::host_not_found;
    while (error && iter != end) {
      ROS_INFO_STREAM("Trying to connect to " << iter->endpoint() << "...");

      socket_->close();
      // Start the synchronous connect operation.
      socket_->connect(*iter++, error);
    }
    if (!error)
      break;
    else if (counter++ > max_attempts) {
      ROS_ERROR("Connection Not Established. Shutting Down.");
      chrono::shutdown();
      ros::shutdown();
    }

    ROS_WARN_STREAM("Attempt " << counter << " Failed. " << std::endl
                               << "Connection Not Established. " << std::endl
                               << "Trying Again.");
    sleep(1);
  }

  ROS_INFO("Connection Established.");

  // Send configuration message to chrono
  ROS_INFO("Sending Configuration Message.");

  flatbuffers::FlatBufferBuilder builder;

  flatbuffers::Offset<ChRosMessage::Config> config =
      ChRosMessage::CreateConfig(builder, use_irrlicht_);
  flatbuffers::Offset<ChRosMessage::Message> message =
      ChRosMessage::CreateMessage(builder, ChRosMessage::Type_Config,
                                  config.Union());

  builder.FinishSizePrefixed(message);
  // Get size of FlatBuffer message in bytes
  int32_t size = builder.GetSize();
  // Get buffer pointer from message object
  uint8_t *buffer = builder.GetBufferPointer();
  // Send the message
  uint32_t send_size = socket_.send(boost::asio::buffer(buffer, size));
  ROS_DEBUG_STREAM("Bytes Sent :: " << send_size);

  // Set up asynchronous sensor callbacks
  if (thread_.joinable()) {
    thread_.join();
  }
  std::thread thr(&ChTransportManager::readSensorData, this);
  thread_ = std::move(thr);
}

void ChTransportManager::readSensorData() {
  uint8_t buffer[4];
  boost::asio::async_read(
      *socket_, boost::asio::buffer(buffer, 4),
      [&](const boost::system::error_code &ec, size_t size)) {
    if (!ec) {
      handleSensorData(((int *)buffer)[0]);
    } else {
      socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
    }
  });
  socket_->get_io_service().run();
}

void ChTransportManager::handleSensorData(size_t size) {
  uint8_t buffer[size];
  boost::asio::async_read(
      *socket_, boost::asio::buffer(buffer, size),
      [&](const boost::system::error_code &ec, size_t size)) {
    if (!ec) {

    } else {
      socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
    }
  });
}
} // namespace transport
} // namespace chrono
