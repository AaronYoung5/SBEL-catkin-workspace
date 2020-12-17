#include "ch_interface/transport/ch_transport_manager.h"

namespace chrono {
namespace transport {
ChTransportManager::ChTransportManager(ros::NodeHandle &n)
    : socket_(std::make_shared<boost::asio::ip::tcp::socket>(
          *new boost::asio::io_service)),
<<<<<<< HEAD
      closed_(false) {
  n.param<std::string>("chrono/hostname", host_name_, "localhost");
  n.param<std::string>("chrono/port", port_num_, "8080");
  establishConnection();
  loadParameters(n);
=======
      resolver_(socket_->get_io_service()) {
  n.param<std::string>("chrono/hostname", host_name_, "localhost");
  n.param<std::string>("chrono/port", port_num_, "8080");
  establishConnection();
  if (ros::ok())
    loadParameters(n);
>>>>>>> d84456083cd1453b3a85c92286e5fa9be41093f3
}

ChTransportManager::~ChTransportManager() {
  // socket_->close();
  // socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
  if (thread_.joinable())
    thread_.join();
}

void ChTransportManager::establishConnection() {
<<<<<<< HEAD
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
=======
  int count = 0;
  async_start(count);
  socket_->get_io_service().run();
  socket_->get_io_service().reset();
  ROS_INFO("Connection Established.");
}

void ChTransportManager::async_start(int &count) {
  // Initialize socket
  ROS_INFO_STREAM("Trying to connect to :: " << host_name_ << ":" << port_num_);
  boost::asio::ip::tcp::resolver::query query(host_name_, port_num_);
  resolver_.async_resolve(
      query, boost::bind(&ChTransportManager::resolve_handler, this,
                         boost::asio::placeholders::error,
                         boost::asio::placeholders::iterator, ++count));
}

void ChTransportManager::resolve_handler(
    const boost::system::error_code &ec,
    boost::asio::ip::tcp::resolver::iterator it, int &count) {
  if (!ec) {
    boost::asio::async_connect(
        *socket_, it,
        boost::bind(&ChTransportManager::connect_handler, this,
                    boost::asio::placeholders::error, ++it, count));
  } else {
    ROS_ERROR("Could Not Resolve Connection. Shutting Down.");
    ros::shutdown();
    return;
  }
}

void ChTransportManager::connect_handler(
    const boost::system::error_code &ec,
    boost::asio::ip::tcp::resolver::iterator it, int &count) {
  if (!ec) {
    ROS_INFO_STREAM("Connection Established With " << it->endpoint() << ".");
    return;
  } else if (it != boost::asio::ip::tcp::resolver::iterator()) {
    ROS_INFO_STREAM("Could not connect to " << it->endpoint()
                                            << ". Trying next endpoint.");
    boost::asio::async_connect(
        *socket_, it,
        boost::bind(&ChTransportManager::connect_handler, this,
                    boost::asio::placeholders::error, ++it, count));
  } else {
    if (count < 5) {
      ROS_WARN_STREAM("Connection Attempt " << count << " Unsuccessful. Trying Again.");
      sleep(1);
      async_start(count);
    } else {
      ROS_ERROR("Connection Not Established. Shutting Down.");
      ros::shutdown();
    }
  }
>>>>>>> d84456083cd1453b3a85c92286e5fa9be41093f3
}

void ChTransportManager::loadParameters(ros::NodeHandle &n) {
  int sim_freq;
  n.param("chrono/simulation_step_size", sim_freq, 1000);

<<<<<<< HEAD
  ros::Publisher pub(n.advertise<rosgraph_msgs::Clock>("clock", 1));
  transports_.push_back(
      ChPublisher(pub, TransportType::TIME, "clock", sim_freq));
=======
  ros::Publisher pub(n.advertise<rosgraph_msgs::Clock>("/clock", 1));
  transports_.push_back(std::make_shared<ChPublisher>(pub, TransportType::TIME,
                                                      "clock", sim_freq));

  ros::Subscriber sub(n.subscribe("/control", 1,
                                  &ChFlatbufferHandler::controlCallback,
                                  &flatbuffer_handler_));
  transports_.push_back(std::make_shared<ChSubscriber>(sub, TransportType::TIME,
                                                       "control", sim_freq));
>>>>>>> d84456083cd1453b3a85c92286e5fa9be41093f3

  const char *sensors[] = {"camera", "lidar", "gps", "imu"};

  flatbuffers::FlatBufferBuilder builder;
  FlatbufferSensorVector flatbuffer_sensors;
  for (const char *sensor : sensors) {
    size_t i = 0;
    while (true) {
      std::stringstream ss;
      ss << sensor << i++;
      std::string id = ss.str();
      std::string param_name = std::string("sensors/") + ss.str();
      if (n.hasParam(param_name)) {
        ROS_WARN_STREAM("param_name " << param_name << " found");
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

          int height;
          n.getParam(param_name + std::string("_height"), height);

          int width;
          n.getParam(param_name + std::string("_width"), width);

          std::vector<float> offset;
          n.getParam(param_name + std::string("_offset"), offset);

          ros::Publisher pub(
              n.advertise<sensor_msgs::Image>(topic_name, queue_size));
<<<<<<< HEAD
          transports_.push_back(
              ChPublisher(pub, TransportType::CAMERA, id, freq));
=======
          transports_.push_back(std::make_shared<ChPublisher>(
              pub, TransportType::CAMERA, id, freq));
>>>>>>> d84456083cd1453b3a85c92286e5fa9be41093f3

          auto camera = ChConfigMessage::CreateCamera(builder, height, width);
          auto sensor = ChConfigMessage::CreateSensor(
              builder, ChConfigMessage::Type_Camera, camera.Union(),
              builder.CreateString(id), freq,
              builder.CreateVector<float>(offset));

          flatbuffer_sensors.push_back(sensor);
        }
      } else {
        ROS_DEBUG_STREAM("param_name " << param_name << " not found");
        break;
      }
    }
  }
  initChrono(builder, flatbuffer_sensors);
}

void ChTransportManager::initChrono(flatbuffers::FlatBufferBuilder &builder,
                                    FlatbufferSensorVector &sensors) {
  // Send configuration message to chrono
  ROS_INFO("Sending Configuration Message.");

  flatbuffers::Offset<ChConfigMessage::Message> message =
      ChConfigMessage::CreateMessage(builder, builder.CreateVector(sensors));

  builder.FinishSizePrefixed(message);
  // Get size of FlatBuffer message in bytes
  int32_t size = builder.GetSize();
  // Get buffer pointer from message object
  uint8_t *buffer = builder.GetBufferPointer();
  // Send the message
<<<<<<< HEAD
  uint32_t send_size = socket_->send(boost::asio::buffer(buffer, size));
  ROS_INFO_STREAM("Bytes Sent :: " << send_size);
=======
  socket_->async_send(boost::asio::buffer(buffer, size),
                      [](const boost::system::error_code &ec, size_t size) {
                        ROS_INFO_STREAM("Bytes Sent :: " << size);
                      });
>>>>>>> d84456083cd1453b3a85c92286e5fa9be41093f3
}

void ChTransportManager::startTransport() {
  // Set up asynchronous sensor callbacks
  if (thread_.joinable()) {
    thread_.join();
  }

<<<<<<< HEAD
  std::thread thr([&, this]() {
    readTransportMessage();
    socket_->get_io_service().run();
  });
=======
  readTransportMessage();
  std::thread thr([&, this]() { socket_->get_io_service().run(); });
>>>>>>> d84456083cd1453b3a85c92286e5fa9be41093f3
  thread_ = std::move(thr);
}

void ChTransportManager::readTransportMessage() {
<<<<<<< HEAD
  buffer_.resize(4);
  boost::asio::async_read(
      *socket_, boost::asio::buffer(buffer_.data(), 4),
      [&](const boost::system::error_code &ec, size_t size) {
        if (!ec) {
          handleTransportMessage(((int *)buffer_.data())[0]);
        } else {
          ROS_ERROR(
              "readTransportMessage() :: Error on async_read. Shutting down.");
=======
  flatbuffer_handler_.resize(sizeof(int));
  boost::asio::async_read(
      *socket_, boost::asio::buffer(flatbuffer_handler_.data(), sizeof(int)),
      [&](const boost::system::error_code &ec, size_t size) {
        if (!ec) {
          handleTransportMessage(flatbuffer_handler_.getPrefixedSize());
        } else {
          ROS_ERROR("readTransportMessage() :: Error on async_read. Shutting "
                    "down.");
          socket_->close();
          ros::shutdown();
>>>>>>> d84456083cd1453b3a85c92286e5fa9be41093f3
        }
      });
  ROS_DEBUG("Beginning Async Listen.");
}

void ChTransportManager::handleTransportMessage(size_t size) {
  ROS_DEBUG("Reading Transport Message.");
  ROS_DEBUG_STREAM("Bytes Available :: " << size);
<<<<<<< HEAD
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

          it->spinOnce(message);
=======
  flatbuffer_handler_.resize(size);
  boost::asio::async_read(
      *socket_, boost::asio::buffer(flatbuffer_handler_.data(), size),
      [&](const boost::system::error_code &ec, size_t size) {
        ROS_DEBUG_STREAM("Bytes Received :: " << size);
        if (!ec) {
          // for (std::shared_ptr<ChTransport> transport : transports_) {
          // transport->spinOnce(flatbuffer_handler_);
          // }

          // const ChInterfaceMessage::Messages *messages =
          //     flatbuffers::GetRoot<ChInterfaceMessage::Messages>(
          //         buffer_.data());
          //
          // for (int i = 0; i < messages->messages()->Length(); i++) {
          //   auto message = static_cast<const ChInterfaceMessage::Message *>(
          //       messages->messages()->Get(i));
          //
          //   // Look for transport with the received id
          //   auto it = std::find_if(
          //       transports_.begin() + 1, transports_.end(),
          //       [message](const std::shared_ptr<ChTransport> t) -> bool {
          //         return t->id().compare(message->id()->str()) == 0;
          //       });
          //
          //   if (it == transports_.end()) {
          //     ROS_WARN("Transport was not found. Ignoring this message.");
          //     continue;
          //   }
          //
          //   std::dynamic_pointer_cast<ChPublisher>((*it))->update(message);
          //   (*it)->spinOnce();
          // }
          // std::dynamic_pointer_cast<ChPublisher>(transports_[0])
          // ->update(messages->time());
          // transports_[0]->spinOnce();
>>>>>>> d84456083cd1453b3a85c92286e5fa9be41093f3

          readTransportMessage();
        } else {
          ROS_ERROR("handleTransportMessage() :: Error on async_read. Shutting "
                    "down.");
        }
      });
}
} // namespace transport
} // namespace chrono
