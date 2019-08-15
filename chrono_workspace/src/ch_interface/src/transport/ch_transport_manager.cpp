#include "ch_interface/transport/ch_transport_manager.h"

namespace chrono {
namespace transport {
ChTransportManager::ChTransportManager(ros::NodeHandle &n)
    : socket_(std::make_shared<boost::asio::ip::tcp::socket>(
          *new boost::asio::io_service)),
      closed_(false) {
  loadParameters(n);
  initChrono();
}

ChTransportManager::~ChTransportManager() {
  // socket_->close();
  // socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
  if (thread_.joinable())
    thread_.join();
}

void ChTransportManager::loadParameters(ros::NodeHandle &n) {
  n.param<std::string>("chrono/hostname", host_name_, "localhost");
  n.param<std::string>("chrono/port", port_num_, "8080");

  int sim_freq;
  n.param("chrono/simulation_step_size", sim_freq, 1000);

  ros::Publisher pub(n.advertise<rosgraph_msgs::Clock>("clock", 1));
  transports_.push_back(ChPublisher(pub, "clock", sim_freq));

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

      switch (sensor_type) {
      case TransportType::CAMERA:

        ros::Publisher pub(
            n.advertise<sensor_msgs::Image>(topic_name, queue_size));
        transports_.push_back(ChPublisher(pub, id, freq));
      }
    }
  }
}

void ChTransportManager::initChrono() {
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
      ros::shutdown();
      return;
    }

    ROS_WARN_STREAM("Connection Attempt " << counter
                                          << " Failed. Trying Again.");
    sleep(1);
  }

  ROS_INFO("Connection Established.");

  // Send configuration message to chrono
  ROS_INFO("Sending Configuration Message.");

  flatbuffers::FlatBufferBuilder builder;

  flatbuffers::Offset<ChInterfaceMessage::Config> config =
      ChInterfaceMessage::CreateConfig(builder);
  flatbuffers::Offset<ChInterfaceMessage::Message> message =
      ChInterfaceMessage::CreateMessage(
          builder, ChInterfaceMessage::Type_Config, config.Union());

  builder.FinishSizePrefixed(message);
  // Get size of FlatBuffer message in bytes
  int32_t size = builder.GetSize();
  // Get buffer pointer from message object
  uint8_t *buffer = builder.GetBufferPointer();
  // Send the message
  uint32_t send_size = socket_->send(boost::asio::buffer(buffer, size));
  ROS_DEBUG_STREAM("Bytes Sent :: " << send_size);
}

void ChTransportManager::startTransport() {
  // Set up asynchronous sensor callbacks
  if (thread_.joinable()) {
    thread_.join();
  }

  std::thread thr([&, this]() {
    readTransportMessage();
    socket_->get_io_service().run();
  });
  thread_ = std::move(thr);
}

void ChTransportManager::readTransportMessage() {
  buffer_.resize(4);
  boost::asio::async_read(
      *socket_, boost::asio::buffer(buffer_.data(), 4),
      [&](const boost::system::error_code &ec, size_t size) {
        if (!ec) {
          handleTransportMessage(((int *)buffer_.data())[0]);
        } else {
          ROS_ERROR(
              "readTransportMessage() :: Error on async_read. Shutting down.");
        }
      });
  ROS_DEBUG("Beginning Async Listen.");
}

void ChTransportManager::handleTransportMessage(size_t size) {
  ROS_DEBUG("Reading Transport Message.");
  ROS_DEBUG_STREAM("Bytes Available :: " << size);
  buffer_.resize(size);
  boost::asio::async_read(
      *socket_, boost::asio::buffer(buffer_.data(), size),
      [&](const boost::system::error_code &ec, size_t size) {
        ROS_DEBUG_STREAM("Bytes Received :: " << size);
        if (!ec) {
          const ChInterfaceMessage::Message *message =
              flatbuffers::GetRoot<ChInterfaceMessage::Message>(buffer_.data());
          // Look for transport with the received id
          auto it =
              std::find_if(transports_.begin(), transports_.end(),
                           [message](const ChTransport &t) -> bool {
                             return t.id().compare(message->id()->str()) == 0;
                           });
          if (it == transports_.end()) {
            ROS_WARN("Transport was not found. Ignoring this message.");
          }

          std::cout << "TESTSETSETSETSETST" << std::endl;

          readTransportMessage();
        } else {
          ROS_ERROR("handleTransportMessage() :: Error on async_read. Shutting "
                    "down.");
        }
      });
}
} // namespace transport
} // namespace chrono