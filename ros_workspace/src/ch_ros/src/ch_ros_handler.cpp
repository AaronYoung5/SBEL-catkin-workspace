// Header include
#include "ch_ros/ch_ros_handler.h"

ChRosHandler::ChRosHandler(ros::NodeHandle &n)
    : socket_(*(new boost::asio::io_service)), lidar_(n, "lidar", 10),
      imu_(n, "imu", 10), gps_(n, "gps", 10), time_(n, "clock", 10),
      cones_(n, "cones", 10), vehicle_(n, "vehicle", 10), chrono_ok_(true),
      throttle_(0), steering_(0), braking_(0),
      control_sub_(
          n.subscribe("control", 10, &ChRosHandler::setTargetControls, this)),
      send_rate_(.01), controls_updated_(false) {
  initializeROSParameters(n);
  initializeSocket();
  if (use_protobuf_)
    protobufSendConfig();
  else
    flatbuffersSendConfig();
}

ChRosHandler::~ChRosHandler() { socket_.close(); }

void ChRosHandler::initializeROSParameters(ros::NodeHandle &n) {
  n.param("/ch_ros/use_protobuf", use_protobuf_, false);
  n.param("/ch_ros/use_irrlicht", use_irrlicht_, false);
  n.param<std::string>("/ch_ros/host_name", host_name_, "localhost");
  n.param<std::string>("/ch_ros/port_num", port_num_, "8080");
}

void ChRosHandler::initializeSocket() {
  // sleep(1);
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

    std::cout << "Connection Not Established." << std::endl;
    std::cout << "Trying Again." << std::endl;
    sleep(1);
  }

  std::cout << "Connection Established." << std::endl;
}

void ChRosHandler::protobufSendConfig() {
  ChronoMessages::config message;

  message.set_visualize(use_irrlicht_);

  // Get size of protobuf message in bytes
  int32_t size = message.ByteSize();
  // Allocate space for the message
  std::vector<uint8_t> buffer(size + 5);
  // Create header with size and message code
  ((int *)buffer.data())[0] = size;
  buffer.data()[4] = ChMessageCode::CONFIG;
  // Serialize the message into the buffer
  message.SerializeToArray(buffer.data() + 5, size);
  // Send the message
  uint32_t send_size =
      socket_.send(boost::asio::buffer(buffer.data(), size + 5));
  // std::cout << "Bytes Sent :: " << send_size << std::endl;
}

void ChRosHandler::flatbuffersSendConfig() {
  // std::cout << "Sending Configuration" << std::endl;
  flatbuffers::FlatBufferBuilder builder;

  flatbuffers::Offset<ChROSMessage::Config> config =
      ChROSMessage::CreateConfig(builder, use_irrlicht_);
  flatbuffers::Offset<ChROSMessage::Message> message =
      ChROSMessage::CreateMessage(builder, ChROSMessage::Type_Config,
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
  if (use_protobuf_)
    protobufReceiveAndHandle();
  else
    flatbufferReceiveAndHandle();
}

void ChRosHandler::protobufReceiveAndHandle() {
  // Allocate space for message "header"
  std::vector<uint8_t> buffer(4);
  // Receive just the "header"
  socket_.receive(boost::asio::buffer(buffer.data(), 4), 0);
  // Check the size of the message to read
  int available = ((int *)buffer.data())[0] + 1;
  // std::cout << "Available :: " << available << std::endl;
  // Allocate space for the message
  buffer.resize(available);
  // Receive and record size of received packet
  // Read allows us to read tcp buffer until all (int)available are received
  int received =
      boost::asio::read(socket_, boost::asio::buffer(buffer.data(), available));
  // std::cout << "Bytes Received :: " << received << std::endl;
  // auto start = std::chrono::high_resolution_clock::now();
  handle(buffer, received);
  // auto end = std::chrono::high_resolution_clock::now();
  //
  // auto duration =
  //     std::chrono::duration_cast<std::chrono::microseconds>(end - start);

  // std::cout << "Time taken by handle: " << duration.count() << "
  // microseconds"
  //           << std::endl;
}

void ChRosHandler::flatbufferReceiveAndHandle() {
  // Allocate space for message "header"
  std::vector<uint8_t> buffer(4);
  // Receive just the "header"
  socket_.receive(boost::asio::buffer(buffer.data(), 4), 0);
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

  // auto start = std::chrono::high_resolution_clock::now();
  const ChROSMessage::Message *message =
      flatbuffers::GetRoot<ChROSMessage::Message>(buffer.data());
  handle(message, received);
  // auto end = std::chrono::high_resolution_clock::now();

  // auto duration =
  // std::chrono::duration_cast<std::chrono::microseconds>(end - start);

  // std::cout << "Time taken by handle: " << duration.count() << "
  // microseconds"
  // << std::endl;
}

void ChRosHandler::handle(std::vector<uint8_t> buffer, int received) {
  protobufSendControls();

  // Determine message type
  switch ((unsigned)buffer.data()[0]) {
  case ChMessageCode::LIDAR:
    lidar_.tcppublish(buffer, received);
    break;
  case ChMessageCode::GPS:
    gps_.publish(buffer, received);
    break;
  case ChMessageCode::IMU:
    imu_.publish(buffer, received);
    break;
  case ChMessageCode::TIME:
    time_.publish(buffer, received);
    break;
  case ChMessageCode::CONE:
    cones_.publish(buffer, received);
    break;
  case ChMessageCode::VEHICLE:
    vehicle_.publish(buffer, received);
    break;
  case ChMessageCode::EXIT:
    chrono_ok_ = false;
    break;
  }

  // if (fmod(ros::Time::now().toSec(), .05) <= 1e-3) {
  // sendControls();
  // }
}

void ChRosHandler::handle(const ChROSMessage::Message *message, int received) {
  flatbuffersSendControls();

  // Determine message type
  switch (message->message_type()) {
  case ChROSMessage::Type_Lidar:
    // std::cout << "Received Lidar Data" << std::endl;
    lidar_.publish(message, received);
    break;
  case ChROSMessage::Type_GPS:
    // std::cout << "Received GPS Data" << std::endl;
    gps_.publish(message, received);
    break;
  case ChROSMessage::Type_IMU:
    // std::cout << "Received IMU Data" << std::endl;
    imu_.publish(message, received);
    break;
  case ChROSMessage::Type_Time:
    // std::cout << "Received Time Data" << std::endl;
    time_.publish(message, received);
    break;
  case ChROSMessage::Type_Cones:
    // std::cout << "Received Cones Data" << std::endl;
    cones_.publish(message, received);
    break;
  case ChROSMessage::Type_Vehicle:
    vehicle_.publish(message, received);
    break;
  case ChROSMessage::Type_Exit:
    // std::cout << "Received Exit Data" << std::endl;
    chrono_ok_ = false;
    break;
  }

  // if (fmod(ros::Time::now().toSec(), .05) <= 1e-3) {
  // sendControls();
  // }
}

bool ChRosHandler::shouldSend() {
  return fmod(time_.GetTime(), send_rate_) <= 1e-3 && controls_updated_;
}

void ChRosHandler::setTargetControls(
    const common_msgs::Control::ConstPtr &msg) {
  throttle_ = msg->throttle;
  steering_ = msg->steering;
  braking_ = msg->braking;
  controls_updated_ = true;
}

void ChRosHandler::protobufSendControls() {
  if (!shouldSend())
    return;

  ChronoMessages::control message;
  message.set_throttle(throttle_);
  message.set_steering(steering_);
  message.set_braking(braking_);

  int32_t size = message.ByteSize();
  std::vector<uint8_t> buffer(size + 5);
  ((int *)buffer.data())[0] = size;
  buffer.data()[4] = ChMessageCode::CONTROL;
  message.SerializeToArray(buffer.data() + 5, size);
  socket_.async_send(boost::asio::buffer(buffer.data(), size + 5),
                     [&](const boost::system::error_code &ec, size_t size) {});
  controls_updated_ = false;
}

void ChRosHandler::flatbuffersSendControls() {
  if (!shouldSend())
    return;

  flatbuffers::FlatBufferBuilder builder;

  flatbuffers::Offset<ChROSMessage::Control> control =
      ChROSMessage::CreateControl(builder, throttle_, steering_, braking_);
  flatbuffers::Offset<ChROSMessage::Message> message =
      ChROSMessage::CreateMessage(builder, ChROSMessage::Type_Control,
                                  control.Union());

  builder.FinishSizePrefixed(message);

  // Get size of FlatBuffer message in bytes
  int32_t size = builder.GetSize();
  // Get buffer pointer from message object
  uint8_t *buffer = builder.GetBufferPointer();
  // Send the message
  socket_.async_send(boost::asio::buffer(buffer, size),
                     [&](const boost::system::error_code &ec, size_t size) {});
  controls_updated_ = false;
}
