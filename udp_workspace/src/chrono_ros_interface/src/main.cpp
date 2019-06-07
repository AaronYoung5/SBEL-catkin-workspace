// ROS includes
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"

// Package includes
#include "chrono_ros_interface/MessageHandler.h"
#include "chrono_ros_interface/MessagePublisher.h"

MessageHandler handler;

ros::Subscriber key_sub;
ros::Subscriber steering_sub;

int main(int argc, char **argv) {
  ros::init(argc, argv, "chrono_ros_com");
  ros::NodeHandle n;

  MessagePublisher pub(n, handler);

  key_sub = n.subscribe<std_msgs::Int8>("key_msgs", 1000, &MessageHandler::keyboardCallback, &handler);

  steering_sub = n.subscribe<std_msgs::Float32>("steering", 1000, &MessageHandler::steeringCallback, &handler);

  try {
    while (ros::ok() and handler.ok()) {
      handler.ReceiveAndHandle();

      pub.PublishAll();

      ros::spinOnce();
    }
  } catch (std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  ros::shutdown();
}
