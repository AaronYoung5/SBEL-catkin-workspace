#include "ch_ros/ch_message_codes.h"
#include "ch_ros/ch_ros_handler.h"
#include <chrono>

// #include <chrono>
// #include <thread>

#define TCP

ChRosHandler::ChRosHandler(ros::NodeHandle n, std::string host_name,
                           std::string port)
    : port_(port),
      socket_(*(new boost::asio::io_service),
              boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                                             std::atoi(port.c_str()))),
      tcpsocket_(*(new boost::asio::io_service)), lidar_(n, "lidar", 10),
      imu_(n, "imu", 10), gps_(n, "gps", 10), time_(n, "clock", 10),
      cones_(n, "cones", 10), vehicle_(n, "vehicle", 10), ok_(true),
      throttle_(0), steering_(0), braking_(0),
      control_(
          n.subscribe("control", 10, &ChRosHandler::setTargetControls, this)),
      send_rate_(.01), controls_updated_(false), host_name_(host_name) {
#ifdef TCP
  initializeSocket();
#endif
}

void ChRosHandler::initializeSocket() {
  sleep(1);

  boost::asio::ip::tcp::resolver resolver(tcpsocket_.get_io_service());
  boost::asio::ip::tcp::resolver::query query(host_name_, port_);
  boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
  boost::asio::ip::tcp::resolver::iterator end;

  boost::system::error_code error = boost::asio::error::host_not_found;
  while (error && iter != end) {
    std::cout << "Trying to connect to " << iter->endpoint() << "..."
              << std::endl;

    tcpsocket_.close();
    // Start the synchronous connect operation.
    tcpsocket_.connect(*iter++, error);
  }

  std::cout << "Connection Established." << std::endl;
}

void ChRosHandler::receiveAndHandle() {
  // Let the UDP stack fill up
  socket_.receive(boost::asio::null_buffers(), 0);
  // Check the size of the buffer
  int available = socket_.available();
  // Allocate space for the message
  std::vector<uint8_t> buffer(available);
  // Receieve and record size of packet
  int received = socket_.receive_from(
      boost::asio::buffer(buffer.data(), available), endpoint_);
  // int code = buffer.data()[0];
  // Handle received message
  handle(buffer, received);
}

void ChRosHandler::protobufReceiveAndHandle() {
  // Allocate space for message "header"
  std::vector<uint8_t> buffer(4);
  // Receive just the "header"
  tcpsocket_.receive(boost::asio::buffer(buffer.data(), 4), 0);
  // Check the size of the message to read
  int available = ((int *)buffer.data())[0] + 1;
  // std::cout << "Available :: " << available << std::endl;
  // Allocate space for the message
  buffer.resize(available);
  // Receive and record size of received packet
  // Read allows us to read tcp buffer until all (int)available are received
  int received = boost::asio::read(
      tcpsocket_, boost::asio::buffer(buffer.data(), available));
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
  tcpsocket_.receive(boost::asio::buffer(buffer.data(), 4), 0);
  // Check the size of the message to read
  int available = ((int *)buffer.data())[0];
  // std::cout << "Available :: " << available << std::endl;
  // Allocate space for the message
  buffer.resize(available);
  // Receive and record size of received packet
  // Read allows us to read tcp buffer until all (int)available are received
  int received = boost::asio::read(
      tcpsocket_, boost::asio::buffer(buffer.data(), available));
  // std::cout << "Bytes Received :: " << received << std::endl;

  // auto start = std::chrono::high_resolution_clock::now();
  const RosMessage::message *message =
      flatbuffers::GetRoot<RosMessage::message>(buffer.data());
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
#ifdef TCP
    lidar_.tcppublish(buffer, received);
#else
    lidar_.publish(buffer, received);
#endif
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
    ok_ = false;
    break;
  }

  // if (fmod(ros::Time::now().toSec(), .05) <= 1e-3) {
  // sendControls();
  // }
}

void ChRosHandler::handle(const RosMessage::message *message, int received) {
  flatbuffersSendControls();

  // Determine message type
  switch (message->type_type()) {
  case RosMessage::Type_lidar:
    // std::cout << "Received Lidar Data" << std::endl;
    lidar_.publish(message, received);
    break;
  case RosMessage::Type_gps:
    // std::cout << "Received GPS Data" << std::endl;
    gps_.publish(message, received);
    break;
  case RosMessage::Type_imu:
    // std::cout << "Received IMU Data" << std::endl;
    imu_.publish(message, received);
    break;
  case RosMessage::Type_time:
    // std::cout << "Received Time Data" << std::endl;
    time_.publish(message, received);
    break;
  case RosMessage::Type_cones:
    // std::cout << "Received Cones Data" << std::endl;
    cones_.publish(message, received);
    break;
  case RosMessage::Type_vehicle:
    vehicle_.publish(message, received);
    break;
  case RosMessage::Type_exit:
    // std::cout << "Received Exit Data" << std::endl;
    ok_ = false;
    break;
  }

  // if (fmod(ros::Time::now().toSec(), .05) <= 1e-3) {
  // sendControls();
  // }
}

bool ChRosHandler::shouldSend() {
  return fmod(time_.GetTime(), send_rate_) <= 1e-3 && controls_updated_;
}

void ChRosHandler::sendControls() {
  ChronoMessages::control message;
  message.set_throttle(throttle_);
  message.set_steering(steering_);
  message.set_braking(braking_);

  int32_t size = message.ByteSize();
  std::vector<uint8_t> buf(size + 1);
  buf.data()[0] = ChMessageCode::CONTROL;
  message.SerializeToArray(buf.data() + 1, size);
  socket_.send_to(boost::asio::buffer(buf.data(), size + 1), endpoint_);
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
  tcpsocket_.async_send(
      boost::asio::buffer(buffer.data(), size + 5),
      [&](const boost::system::error_code &ec, size_t size) {});
  controls_updated_ = false;
}

void ChRosHandler::flatbuffersSendControls() {
  if (!shouldSend())
    return;

  flatbuffers::FlatBufferBuilder builder;

  flatbuffers::Offset<RosMessage::control> control =
      RosMessage::Createcontrol(builder, throttle_, steering_, braking_);
  flatbuffers::Offset<RosMessage::message> message = RosMessage::Createmessage(
      builder, RosMessage::Type_control, control.Union());

  builder.FinishSizePrefixed(message);

  // Get size of FlatBuffer message in bytes
  int32_t size = builder.GetSize();
  // Get buffer pointer from message object
  uint8_t *buffer = builder.GetBufferPointer();
  // Send the message
  tcpsocket_.async_send(
      boost::asio::buffer(buffer, size),
      [&](const boost::system::error_code &ec, size_t size) {});
  controls_updated_ = false;
}
