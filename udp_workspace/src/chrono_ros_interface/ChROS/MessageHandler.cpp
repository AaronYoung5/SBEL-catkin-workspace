#include "chrono_ros_interface/MessageHandler.h"

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42

MessageHandler::MessageHandler(const char *port_num)
    : m_light_offset(8.0), m_port_num(port_num),
      m_socket(*(new boost::asio::io_service),
               boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                                              std::atoi(m_port_num))),
      m_lidar_all_received(false), m_is_ok(true) {
  // m_publiser = new MessagePublisher(n, &this);
}

void MessageHandler::ReceiveAndHandle() {
  // This is a bit of a tricky process, but it lets us send messages of
  // whatever length we want without having to send the size with each
  // message.
  // Let the UDP stack fill up
  m_socket.receive(boost::asio::null_buffers(), 0);
  // Check the size of the waiting buffer
  int available = m_socket.available();
  // Allocate enough space
  std::vector<uint8_t> buffer(available);
  // Receive and record the actual size of the received packet
  int received = m_socket.receive_from(
      boost::asio::buffer(buffer.data(), available), m_simEndpoint);
  Handle(buffer, received);
}

void MessageHandler::Handle(std::vector<uint8_t> buffer, int received) {
  // std::cout << (unsigned)buffer.data()[0] << std::endl;
  // std::cout << (unsigned)buffer.data()[1] << std::endl;
  switch ((unsigned)buffer.data()[0]) {
  case ProtobufCodes::LIDAR: {
    enum LidarCodes { FIRST, MIDDLE, LAST, FIRST_LAST };
    // std::cout << (unsigned)buffer.data()[1] << std::endl;
    switch ((unsigned)buffer.data()[1]) {
    case FIRST:
      m_lidar = {};
      m_lidar_all_received = false;
      break;
    case LAST:
      m_lidar_all_received = true;
      break;
    case FIRST_LAST:
      m_lidar_all_received = true;
      m_lidar = {};
      break;
    }
    // Parse from what we ended up receiving
    DriverMessages::lidar message;
    message.ParseFromArray(buffer.data() + 2, received - 2);

    for (int i = 0; i < message.points_size(); i++) {
      Vector3D p{message.points(i).x(), message.points(i).y(),
                 message.points(i).z()};

      m_lidar.push_back(p);
    }
    break;
  }
  case ProtobufCodes::GPS: {
    // Parse from what we ended up receiving
    DriverMessages::gps message;
    message.ParseFromArray(buffer.data() + 1, received - 1);
    m_gps.latitude = message.latitude();
    m_gps.longitude = message.longitude();
    m_gps.altitude = message.altitude();
    break;
  }
  case ProtobufCodes::IMU: {
    // Parse from what we ended up receiving
    DriverMessages::imu message;
    message.ParseFromArray(buffer.data() + 1, received - 1);
    m_imu.linear_acceleration.x = message.linear_acceleration().x();
    m_imu.linear_acceleration.y = message.linear_acceleration().y();
    m_imu.linear_acceleration.z = message.linear_acceleration().z();

    m_imu.angular_velocity.x = message.angular_velocity().x();
    m_imu.angular_velocity.y = message.angular_velocity().y();
    m_imu.angular_velocity.z = message.angular_velocity().z();
    break;
  }
  case ProtobufCodes::VEHICLE: {
    DriverMessages::vehicle message;
    message.ParseFromArray(buffer.data() + 1, received - 1);
    m_pos.x = message.x();
    m_pos.y = message.y();
    m_pos.z = message.z();
    break;
  }
  case ProtobufCodes::TIME: {
    // Parse from what we ended up receiving
    DriverMessages::time message;
    message.ParseFromArray(buffer.data() + 1, received - 1);
    m_time = message.t();
    break;
  }
  case ProtobufCodes::LIGHT: {
    DriverMessages::light message;
    message.ParseFromArray(buffer.data() + 1, received - 1);

    float xPos = ((message.xpos() - m_refPosition.longitude) / m_radTodeg) *
                     m_earthRadius * cos(message.xpos() / m_radTodeg) -
                 m_light_offset;
    float yPos = ((message.ypos() - m_refPosition.latitude) / m_radTodeg) *
                 m_earthRadius;

    int xDir = message.xdir();
    int yDir = message.ydir();

    m_light.xPos = xPos;
    m_light.yPos = yPos;
    m_light.xDir = xDir;
    m_light.yDir = yDir;
    m_light = Light{xPos, yPos, xDir, yDir, m_light_offset};
    break;
  }
  case ProtobufCodes::CONE: {
    DriverMessages::cones message;
    message.ParseFromArray(buffer.data() + 1, received - 1);
    m_blue_cones = m_yellow_cones = {};
    for (int i = 0; i < message.blue_cones_size(); i++) {
      Vector3D bp{message.blue_cones(i).x(), message.blue_cones(i).y(),
                  message.blue_cones(i).z()};
      Vector3D yp{message.yellow_cones(i).x(), message.yellow_cones(i).y(),
                  message.yellow_cones(i).z()};

      m_blue_cones.push_back(bp);
      m_yellow_cones.push_back(yp);
    }
    break;
  }
  case ProtobufCodes::EXIT: {
    m_is_ok = false;
    break;
  }
  }

  Send();
}

void MessageHandler::Send() {
  // m_throttle = 1;
  // m_steering = -.05;

  // package and send the control message
  DriverMessages::control message;
  message.set_throttle(m_throttle);
  message.set_steering(m_steering);
  message.set_braking(m_braking);

  int32_t sizeControl = message.ByteSize();
  std::vector<uint8_t> bufferControl(sizeControl + 1);
  bufferControl.data()[0] = ProtobufCodes::CONTROL;
  message.SerializeToArray(bufferControl.data() + 1, sizeControl);
  m_socket.send_to(boost::asio::buffer(bufferControl.data(), sizeControl + 1),
                   m_simEndpoint);
}

void MessageHandler::keyboardCallback(const std_msgs::Int8::ConstPtr &msg) {
  switch (msg->data) {
  case KEYCODE_R:
    TurnRight();
    break;
  case KEYCODE_L:
    TurnLeft();
    break;
  case KEYCODE_U:
    IncreaseThrottle();
    break;
  case KEYCODE_D:
    DecreaseThrottle();
    break;
  }
}

void MessageHandler::steeringCallback(const std_msgs::Float32::ConstPtr &msg) {
  if (msg-> data < 1) {
    TurnLeft();
  }
  else {
    TurnRight();
  }
}
