#include "arduino_interface/serial_handler.h"

SerialHandler::SerialHandler(ros::NodeHandle &n)
    : controls_sub_(
          n.subscribe("control", 1, &SerialHandler::controlsCallback, this)) {}

SerialHandler::~SerialHandler() { serial_.close(); }

int SerialHandler::initSerial() {
  try {
    serial_.setPort("/dev/ttyACM0");
    serial_.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    serial_.setTimeout(to);
    serial_.open();
  } catch (serial::IOException &e) {
    std::cout << "Serial Port Initialization Unsuccessful" << std::endl;
    serial_.close();
    ros::shutdown();
    return -1;
  }

  if (serial_.isOpen()) {
    std::cout << "Serial Port Initializated" << std::endl;
  } else {
    serial_.close();
    ros::shutdown();
    return -1;
  }
}

void SerialHandler::establishConnection() {
  serial_.flush();
  while (serial_.available() <= 0) {
    ros::Duration(1).sleep();
  }
  serial_.readline();

  serial_.flush();
  ros::Duration(3).sleep();
  serial_.write("1");
  std::cout << "Connection Established" << std::endl;

  ros::Duration(3).sleep();
  serial_.flush();
  sendControls();
}

void SerialHandler::sendControls() {
  int size = sizeof(message_);
  uint8_t buffer[size + 4];
  memcpy(buffer + 4, &message_, size);
  buffer[0] = size;
  serial_.flush();
  serial_.write(buffer, size);
  std::cout << "Motor Sent :: "
            << (*(struct ControlMessage *)(buffer + 4)).throttle << std::endl;
  std::cout << "Steering Sent :: "
            << (*(struct ControlMessage *)(buffer + 4)).steering << std::endl;

            serial_.readline();
}

void SerialHandler::controlsCallback(
    const common_msgs::Control::ConstPtr &msg) {
  message_ = ControlMessage{msg->throttle, msg->steering};
  sendControls();
}

int SerialHandler::spin() {
  if (initSerial() == -1)
    return -1;
  ros::Duration(5).sleep();
  establishConnection();

  serial_.flush();
  while (ros::ok()) {
    if (serial_.available()) {
      // serial_.flush();
      std::string msg = serial_.readline();
      std::cout << "Data Received :: " << msg << std::endl;
    }
    ros::spinOnce();
  }
  message_ = ControlMessage{0, 0};
  sendControls();
  return 0;
}
