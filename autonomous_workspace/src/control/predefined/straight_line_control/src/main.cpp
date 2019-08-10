#include "ros/ros.h"

#include "common_msgs/Control.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "straight_line");

  ros::NodeHandle n;
  ros::Duration(5).sleep();
  // what msg file are we advertising to, and what type is our msg? what is the
  // first parameter for?
  ros::Publisher chatter_pub = n.advertise<common_msgs::Control>("control", 1);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {
    // write update for msg??
    common_msgs::Control msg;
    msg.steering = -.05;
    if (count < 30)
      msg.throttle = 0.12;
    else {
      msg.throttle = 0;
      chatter_pub.publish(msg);
      ros::shutdown();
    }
    // insert what msg we need to publish
    chatter_pub.publish(msg);

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
