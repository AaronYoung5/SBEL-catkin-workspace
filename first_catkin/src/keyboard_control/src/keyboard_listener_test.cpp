#include "ros/ros.h"
#include "std_msgs/Int8.h"

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

ros::Subscriber sub;

void callback(const std_msgs::Int8::ConstPtr &msg) {
  switch (msg->data) {
  case KEYCODE_R:
    ROS_INFO("RIGHT");
    break;
  }
  // ROS_INFO("Message recieved :: %s", msg->data.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "keyboard_listener_test");
  ros::NodeHandle n;

  sub = n.subscribe<std_msgs::Int8>("key_msgs", 1000, callback);

  ros::spin();
}
