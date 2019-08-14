#include "ros/ros.h"
#include "std_msgs/String.h"

//ros::Publisher pub;
ros::Subscriber sub;

void callback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Message recieved :: %s", msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "teleop_listener");
  ros::NodeHandle n;
  //pub = n.advertise<std_msgs::String>("teleop_listener_chatter",1000);
  sub = n.subscribe<std_msgs::String>("teleop_chatter", 1000, callback);

  ros::spin();
}
