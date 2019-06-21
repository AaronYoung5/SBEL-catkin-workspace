// ROS includes
#include "ros/ros.h"

// External package includes
#include <boost/program_options.hpp>

// Internal package includes
#include "ch_ros/ch_ros_handler.h"

#define TCP

void exit(int signal) { ros::shutdown(); }

std::vector<std::string> parse_command_line(int argc, char **argv) {
  boost::program_options::options_description desc("Allowed options");
  int opt;
  desc.add_options()("help", "produce help message")(
      "ip,i",
      boost::program_options::value<std::string>()->default_value("localhost"),
      "ip address of chrono")(
      "port,p",
      boost::program_options::value<std::string>()->default_value("8080"),
      "port number");

  boost::program_options::variables_map v_map;
  boost::program_options::store(
      boost::program_options::parse_command_line(argc, argv, desc), v_map);
  boost::program_options::notify(v_map);

  if (v_map.count("help")) {
    std::cout << desc << std::endl;
    exit(0);
  }

  std::vector<std::string> options;

  if (v_map.count("ip")) {
    options.push_back(v_map["ip"].as<std::string>());
    // std::cout << v_map["ip"].as<std::string>() << std::endl;
  }

  if (v_map.count("port")) {
    options.push_back(v_map["port"].as<std::string>());
    // std::cout << v_map["port"].as<std::string>() << std::endl;
  }

  return options;
}

int main(int argc, char **argv) {
  try {
    std::vector<std::string> opts = parse_command_line(argc, argv);

    ros::init(argc, argv, "ch_ros");
    ros::NodeHandle n;

    ChRosHandler handler(n, opts[0], opts[1]);

    signal(SIGINT, exit);

    while (ros::ok() && handler.ok()) {
// #ifdef TCP
      handler.tcpReceiveAndHandle();
// #else
      // handler.receiveAndHandle();
// #endif

      ros::spinOnce();
    }
  } catch (std::exception &e) {
    std::cerr << "Error :: " << e.what() << std::endl;
  }

  ros::shutdown();
}
