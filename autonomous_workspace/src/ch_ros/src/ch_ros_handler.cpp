// Header include
#include "ch_ros/ch_ros_handler.h"

ChRosHandler::ChRosHandler(ros::NodeHandle &n)
    : socket_(*(new boost::asio::io_service)), camera_(n, "camera", 1),
      lidar_(n, "lidar", 1), imu_(n, "imu", 1), gps_(n, "gps", 1),
      time_(n, "clock", 1), cones_(n, "cones", 1), vehicle_(n, "vehicle", 1),
      chrono_ok_(true), throttle_(0), steering_(0), braking_(0) {
  std::string control_topic;
  n.param<std::string>("/control_topic", control_topic, "control");
  control_sub_ =
      n.subscribe(control_topic, 1, &ChRosHandler::setTargetControls, this);
  initializeROSParameters(n);
  initializeSocket();
  sendConfig();
}

ChRosHandler::~ChRosHandler() { socket_.close(); }

void ChRosHandler::initializeROSParameters(ros::NodeHandle &n) {
  n.param("use_irrlicht", use_irrlicht_, true);
  n.param<std::string>("hostname", host_name_, "localhost");
  n.param<std::string>("port", port_num_, "8080");
}

void ChRosHandler::initializeSocket() {
  // sleep(1);
  int counter = 0;
  while (true) {
    boost::asio::ip::tcp::resolver resolver(socket_.get_io_service());
    boost::asio::ip::tcp::resolver::query query(host_name_, port_num_);
    boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
    boost::asio::ip::tcp::resolver::iterator end;

    boost::system::error_code error = boost::asio::error::host_not_found;
    while (error && iter != end) {
      std::cout << "Trying to connect to " << iter->endpoint() << "..."
                << std::endl;

      socket_.close();
      // Start the synchronous connect operation.
      socket_.connect(*iter++, error);
    }
    if (!error)
      break;
    else if (counter++ > 5) {
      std::cout << "Connection Not Established" << std::endl;
      std::cout << "Shutting Down" << std::endl;
      break;
      ros::shutdown();
    }

    std::cout << counter << std::endl;

    std::cout << "Connection Not Established." << std::endl;
    std::cout << "Trying Again." << std::endl;
    sleep(1);
  }

  std::cout << "Connection Established." << std::endl;
}

void ChRosHandler::sendConfig() {
  // std::cout << "Sending Configuration" << std::endl;
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
  // std::cout << "Bytes Sent :: " << send_size << std::endl;
  sleep(1);
}

void ChRosHandler::receive() {
  // Allocate space for message "header"
  std::vector<uint8_t> buffer(4);
  // Receive just the "header"
  boost::asio::read(socket_, boost::asio::buffer(buffer.data(), 4));
  // Check the size of the message to read
  int available = ((int *)buffer.data())[0];
  // std::cout << "Available :: " << available << std::endl;
  // Allocate space for the message
  buffer.resize(available);
  // Receive and record size of received packet
  // Read allows us to read tcp buffer until all (int)available are received
  int received =
      boost::asio::read(socket_, boost::asio::buffer(buffer.data(), available));
  // std::cout << "Bytes Received :: " << received << std::endl;

  const ChRosMessage::Message *message =
      flatbuffers::GetRoot<ChRosMessage::Message>(buffer.data());
  handle(message, received);
}

void ChRosHandler::handle(const ChRosMessage::Message *message, int received) {
  // sendControls();
  // Determine message type
  switch (message->message_type()) {
  case ChRosMessage::Type_Camera:
    // std::cout << "Received Lidar Data" << std::endl;
    camera_.publish(message, received);
    break;
  case ChRosMessage::Type_Lidar:
    // std::cout << "Received Lidar Data" << std::endl;
    lidar_.publish(message, received);
    break;
  case ChRosMessage::Type_GPS:
    // std::cout << "Received GPS Data" << std::endl;
    gps_.publish(message, received);
    break;
  case ChRosMessage::Type_IMU:
    // std::cout << "Received IMU Data" << std::endl;
    imu_.publish(message, received);
    break;
  case ChRosMessage::Type_Time:
    // std::cout << "Received Time Data" << std::endl;
    time_.publish(message, received);
    break;
  case ChRosMessage::Type_Cones:
    // std::cout << "Received Cones Data" << std::endl;
    cones_.publish(message, received);
    break;
  case ChRosMessage::Type_Vehicle:
    vehicle_.publish(message, received);
    break;
  case ChRosMessage::Type_Exit:
    // std::cout << "Received Exit Data" << std::endl;
    chrono_ok_ = false;
    break;
  }
}

void ChRosHandler::setTargetControls(
    const common_msgs::Control::ConstPtr &msg) {
  throttle_ = msg->throttle;
  steering_ = msg->steering;
  braking_ = msg->braking;

  sendControls();
}

void ChRosHandler::sendControls() {
  flatbuffers::FlatBufferBuilder builder;

  flatbuffers::Offset<ChRosMessage::Control> control =
      ChRosMessage::CreateControl(builder, throttle_, steering_, braking_);
  flatbuffers::Offset<ChRosMessage::Message> message =
      ChRosMessage::CreateMessage(builder, ChRosMessage::Type_Control,
                                  control.Union());

  builder.FinishSizePrefixed(message);

  // Get size of FlatBuffer message in bytes
  int32_t size = builder.GetSize();
  // Get buffer pointer from message object
  uint8_t *buffer = builder.GetBufferPointer();
  // Send the message
  socket_.async_send(boost::asio::buffer(buffer, size),
                     [&](const boost::system::error_code &ec, size_t size) {});
}
