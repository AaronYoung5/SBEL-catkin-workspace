#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"

std::vector<float> path;
float pos[3];
bool pos_found = false;
bool path_recieved = false;

void pathCallback(const std_msgs::Float32MultiArray::ConstPtr &msg) {
  path_recieved = true;
  for (std::vector<float>::const_iterator it = msg->data.begin();
       it != msg->data.end(); ++it) {
    path.push_back(*it);
  }
}

void gpsCallback(const std_msgs::Float32MultiArray::ConstPtr &msg) {
  int i = 0;
  for (std::vector<float>::const_iterator it = msg->data.begin();
       it != msg->data.end(); ++it) {
    pos[i] = *it;
    i++;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_follower_controller");
  ros::NodeHandle n;

  ros::Subscriber path_sub = n.subscribe<std_msgs::Float32MultiArray>("path_msg", 1, pathCallback);
  ros::Subscriber gps_sub = n.subscribe<std_msgs::Float32MultiArray>("veh_gps", 1000, gpsCallback);

  ros::Publisher steering_pub = n.advertise<std_msgs::Float32>("steer_control", 1000);
  ros::Publisher throttle_pub = n.advertise<std_msgs::Float32>("throttle_control", 1000);
  ros::Publisher braking_pub = n.advertise<std_msgs::Float32>("braking_control", 1000);

  ros::Rate r(60);

  while(ros::ok()) {




    r.sleep();
  }
}
