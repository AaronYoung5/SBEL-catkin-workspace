#include "ch_ros/ch_message_codes.h"
#include "ch_ros/ch_ros_handler.h"

ChRosHandler::ChRosHandler(ros::NodeHandle n, const char *port_num)
    : port_num_(port_num),
      socket_(*(new boost::asio::io_service),
              boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                                             std::atoi(port_num))),
      lidar_(n, "lidar", 10), imu_(n, "imu", 10), gps_(n, "gps", 10),
      time_(n, "clock", 10), cones_(n, "cones", 10), ok_(true) {}

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

void ChRosHandler::handle(std::vector<uint8_t> buffer, int received) {
  // Determine message type
  switch ((unsigned)buffer.data()[0]) {
  case ChMessageCode::LIDAR:
    lidar_.publish(buffer, received);
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

  sendControls();
}

void ChRosHandler::sendControls() {
  // package and send the control message
  ChronoMessages::control message;
  message.set_throttle(0);
  message.set_steering(0);
  message.set_braking(1);

  int32_t size = message.ByteSize();
  std::vector<uint8_t> buf(size + 1);
  buf.data()[0] = ChMessageCode::CONTROL;
  message.SerializeToArray(buf.data() + 1, size);
  socket_.send_to(boost::asio::buffer(buf.data(), size + 1), endpoint_);
}
