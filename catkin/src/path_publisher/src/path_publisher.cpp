#include "ros/ros.h"
#include "path_publisher/path_msg.h"
#include "geometry_msgs/Point.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_publisher");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<path_publisher::path_msg>("path_msg", 1000);

  path_publisher::path_msg path;
  int num = 20;

  geometry_msgs::Point p;
  for(int i = 0; i < num; i++) {
    p.x = i;
    p.y = 0;
    path.points.push_back(p);
  }

  ros::Rate r(60);

  while(ros::ok()) {
    if(pub.getNumSubscribers() >= 1) {
      pub.publish(path);
    }

    ros::spinOnce();
    r.sleep();
  }
}
