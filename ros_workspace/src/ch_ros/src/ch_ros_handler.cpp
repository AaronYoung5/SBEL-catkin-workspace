#include "ch_ros/ch_message_codes.h"
#include "ch_ros/ch_ros_handler.h"

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
      cones_(n, "cones", 10), ok_(true), throttle_(0), steering_(0),
      braking_(0), control_(n.subscribe(
                       "control", 10, &ChRosHandler::setTargetControls, this)),
      host_name_(host_name) {
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
  // Handle received message
  handle(buffer, received);
}

void ChRosHandler::tcpReceiveAndHandle() {
  // Allocate space for message "header"
  std::vector<uint8_t> buffer(4);
  // Receive just the "header"
  tcpsocket_.receive(boost::asio::buffer(buffer.data(), 4), 0);
  // Check the size of the message to read
  int available = ((int *)buffer.data())[0] + 1;
  // Allocate space for the message
  buffer.resize(available);
  // Receive and record size of received packet
  // Read allows us to read tcp buffer until all (int)available are received
  int received = boost::asio::read(
      tcpsocket_, boost::asio::buffer(buffer.data(), available));
  std::cout << "Bytes Received :: " << received << std::endl;
  handle(buffer, received);
}

void ChRosHandler::handle(std::vector<uint8_t> buffer, int received) {
#ifdef TCP
  // tcpSendControls();
#else
  sendControls();
#endif
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
  case ChMessageCode::EXIT:
    ok_ = false;
    break;
  }

  // if (fmod(ros::Time::now().toSec(), .05) <= 1e-3) {
  // sendControls();
  // }
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
  throttle_ = msg->throttle.data;
  steering_ = msg->steering.data;
  braking_ = msg->braking.data;
}

void ChRosHandler::tcpSendControls() {
  ChronoMessages::control message;
  message.set_throttle(throttle_);
  message.set_steering(steering_);
  message.set_braking(braking_);

  int32_t size = message.ByteSize();
  std::vector<uint8_t> buf(size + 1);
  buf.data()[0] = ChMessageCode::CONTROL;
  message.SerializeToArray(buf.data() + 1, size);
  tcpsocket_.send(boost::asio::buffer(buf.data(), size + 1));
}
