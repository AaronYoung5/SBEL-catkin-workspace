#include "ch_ros/ch_message_codes.h"
#include "ch_ros/ch_ros_handler.h"

#include <chrono>
#include <thread>

ChRosHandler::ChRosHandler(ros::NodeHandle n, const char *port_num)
    : port_num_(port_num),
      socket_(*(new boost::asio::io_service),
              boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                                             std::atoi("port_num"))),
      tcpsocket_(*(new boost::asio::io_service)), lidar_(n, "lidar", 10),
      imu_(n, "imu", 10), gps_(n, "gps", 10), time_(n, "clock", 10),
      cones_(n, "cones", 10), ok_(true), throttle_(0), steering_(0),
      braking_(0), control_(n.subscribe(
                       "control", 10, &ChRosHandler::setTargetControls, this)) {
  // boost::asio::ip::tcp::resolver resolver(tcpsocket_.get_io_service());
  // boost::asio::ip::tcp::resolver::query query("localhost", "test");
  // boost::asio::ip::tcp::resolver::iterator endpoint_iterator =
  // resolver.resolve(query);
  std::chrono::milliseconds dura(1000);
  std::this_thread::sleep_for(dura);
  tcpsocket_.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(),
                                                    std::atoi(port_num)));
  const std::string msg = "Hello from Client!";
  boost::system::error_code error;
  tcpsocket_.send(boost::asio::buffer(msg), 0, error);
  if (!error) {
    std::cout << "Successfully sent message to Chrono" << std::endl;
  } else {
    std::cout << "Failed to send message to Chrono" << std::endl;
  }
}

void ChRosHandler::receiveAndHandle() {
  // auto start = std::chrono::high_resolution_clock::now();

  // Let the UDP stack fill up
  socket_.receive(boost::asio::null_buffers(), 0);

  // auto end = std::chrono::high_resolution_clock::now();

  // auto duration =
  // std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  // Check the size of the buffer
  int available = socket_.available();
  // Allocate space for the message
  std::vector<uint8_t> buffer(available);
  // Receieve and record size of packet
  int received = socket_.receive_from(
      boost::asio::buffer(buffer.data(), available), endpoint_);
  // Handle received message
  handle(buffer, received);

  // if (duration.count() > 0) {
  // std::cout << "Time taken by received: " << duration.count()
  //           << " microseconds" << std::endl;
  // std::cout << "Time taken by received: " << (duration.count() * 1e-6)
  //           << " seconds" << std::endl;
  // std::cout << "Message Code: " << (unsigned)buffer.data()[0] << std::endl;
  // }
}

void ChRosHandler::tcpReceiveAndHandle() {
  // Let the TCP stack fill up
  tcpsocket_.receive(boost::asio::null_buffers(), 0);
  // Check the size of the buffer
  int available = tcpsocket_.available();
  // Allocate space for the message
  std::vector<uint8_t> buffer(available);
  // Receieve and record size of packet
  int received =
      tcpsocket_.receive(boost::asio::buffer(buffer.data(), available), 0);
  handle(buffer, received);
}

void ChRosHandler::handle(std::vector<uint8_t> buffer, int received) {
  // Determine message type
  switch ((unsigned)buffer.data()[0]) {
  case ChMessageCode::LIDAR:
    // lidar_.publish(buffer, received);
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
  tcpSendControls();
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
