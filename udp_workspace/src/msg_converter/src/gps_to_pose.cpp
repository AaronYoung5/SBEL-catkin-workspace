#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/NavSatFix.h"

ros::Subscriber gps_sub;
ros::Publisher pose_pub;

sensor_msgs::NavSatFix initial_location{};

void GPSToPose(const sensor_msgs::NavSatFix::ConstPtr &msg) {
  if (initial_location.altitude == 0 && initial_location.longitude == 0 &&
      initial_location.latitude == 0) {
    initial_location = *msg;
  } else {
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "veh_pose";

    pose.pose.position.x = msg->latitude - initial_location.latitude;
    pose.pose.position.y = msg->longitude - initial_location.longitude;
    pose.pose.position.z = msg->altitude - initial_location.altitude;

    pose_pub.publish(pose);
  }
}

  int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_pose");
    ros::NodeHandle n;

    gps_sub = n.subscribe<sensor_msgs::NavSatFix>("gps", 1000, GPSToPose);

    pose_pub = n.advertise<geometry_msgs::PoseStamped>("veh_pose", 1000);

    ros::spin();
  }
