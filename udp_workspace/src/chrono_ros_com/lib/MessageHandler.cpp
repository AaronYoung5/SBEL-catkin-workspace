#include "chrono_ros_com/MessageHandler.h"

MessageHandler::MessageHandler(const char *port_num)
    : m_lightOffset(8.0), m_port_num(port_num),
      m_socket(*(new boost::asio::io_service),
               boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                                              std::atoi(m_port_num))) {}

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
  switch ((unsigned)buffer.data()[0]) {
  // case LIDAR_MESSAGE: {
  //   // Parse from what we ended up receiving
  //   DriverMessages::lidar message;
  //   message.ParseFromArray(buffer.data() + 1, received - 1);
  //   std::vector<double> old_lidarData = m_lidarData;
  //   m_lidarData = {};
  //   // std::cout << "LiDAR data :" << std::endl;
  //   for (int i = 0; i < message.data_size(); i++) {
  //     // std::cout << message.data(i) << " ";
  //     m_lidarData.push_back(message.data(i));
  //   }
  //
  //   // if (m_lidarData == old_lidarData) {
  //   //     std::cout << "Both vectors are equal" << std::endl;
  //   // } else {
  //   //     std::cout << "Both vectors are not equal" << std::endl;
  //   // }
  //   // std::endl(std::cout);
  //   break;
  // }
  case LIDAR_MESSAGE: {
    // Parse from what we ended up receiving
    DriverMessages::lidar lidarMessage;
    lidarMessage.ParseFromArray(buffer.data() + 1, received - 1);
    m_lidarData = {};
    for (int i = 0; i < lidarMessage.points_size(); i++) {
      Position p(lidarMessage.points(i).x(), lidarMessage.points(i).y(),
                 lidarMessage.points(i).z());
      // std::cout << "(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
      m_lidarData.push_back(p);
    }
    break;
  }
  case GPS_MESSAGE: {
    // Parse from what we ended up receiving
    DriverMessages::gps message;
    message.ParseFromArray(buffer.data() + 1, received - 1);
    m_position = ToRelativePosition(message);
    // std::cout << "GPS data [Longitude, Latitude, Altitude]: [" << m_lon << ",
    // "
    //           << m_lat << ", " << m_alt << "]" << std::endl;
    break;
  }
  case IMU_MESSAGE: {
    // Parse from what we ended up receiving
    DriverMessages::imu message;
    message.ParseFromArray(buffer.data() + 1, received - 1);
    m_orientation = ToQuaternion(message);
    // std::cout << "IMU data [e0,e1,e2,e3]: [" << m_e0 << ", " << m_e1 <<
    // ", "
    //           << m_e2 << ", " << m_e3 << "]" << std::endl;
    break;
  }
  case TIME_MESSAGE: {
    // Parse from what we ended up receiving
    DriverMessages::time message;
    message.ParseFromArray(buffer.data() + 1, received - 1);
    m_time = message.t();
    // std::cout << "Time data: " << m_time << std::endl;
    break;
  }
  case LIGHT_MESSAGE: {
    DriverMessages::light message;
    message.ParseFromArray(buffer.data() + 1, received - 1);

    m_lightXPos = ((message.xpos() - m_refPosition.lon) / m_radTodeg) *
                      m_earthRadius * cos(message.xpos() / m_radTodeg) -
                  m_lightOffset;
    m_lightYPos =
        ((message.ypos() - m_refPosition.lat) / m_radTodeg) * m_earthRadius;

    m_lightXDir = message.xdir();
    m_lightYDir = message.ydir();
    // std::cout << "Light data [XPose, YPose, XDir, YDir]: [" << m_lightXPos
    //           << ", " << m_lightYPos << ", " << m_lightXDir << ", "
    //           << m_lightYDir << "]" << std::endl;
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
  bufferControl.data()[0] = CONTROL_MESSAGE;
  message.SerializeToArray(bufferControl.data() + 1, sizeControl);
  m_socket.send_to(boost::asio::buffer(bufferControl.data(), sizeControl + 1),
                   m_simEndpoint);
}

Position MessageHandler::ToRelativePosition(DriverMessages::gps message) {
  double alt = message.altitude();
  double lat = message.latitude();
  double lon = message.longitude();

  double r = m_earthRadius + alt;

  Position p;
  p.x = (lon - m_refPosition.lon) / m_radTodeg * r * cos(lat * m_radTodeg);
  p.y = (lat - m_refPosition.lat) / m_radTodeg * r;
  p.z = alt - m_refPosition.alt;
  return p;
}

Quaternion MessageHandler::ToQuaternion(DriverMessages::imu message) {
  double yaw = message.yaw();
  double roll = message.roll();
  double pitch = message.pitch();

  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  Quaternion q;
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;
  return q;
}
