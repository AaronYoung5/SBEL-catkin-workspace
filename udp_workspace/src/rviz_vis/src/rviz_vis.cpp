#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

geometry_msgs::Point pos;
geometry_msgs::Quaternion ori;
bool pos_found = false;

void gps_update(const geometry_msgs::Pose::ConstPtr &coord) {
  pos_found = true;
  ROS_INFO_ONCE("GPS RECIEVED");

  pos = coord->position;
  ori = coord->orientation;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rviz_vis");
  ros::NodeHandle n;

  ros::Subscriber gps_sub =
      n.subscribe<geometry_msgs::Pose>("gps_msgs", 1000, gps_update);

  ros::Publisher pub =
      n.advertise<visualization_msgs::Marker>("rviz_vis", 1000);

  ros::Rate r(30);

  while (ros::ok()) {
    ros::spinOnce();
    if (!(pos_found)) {
      ROS_WARN_ONCE("Please provide a car position");
      continue;
    }

    visualization_msgs::Marker veh, path_points;

    veh.header.frame_id = /* path_points.header.frame_id = */ "/rv_frame";
    veh.header.stamp = /* path_points.header.stamp = */ ros::Time::now();
    veh.ns = /* path_points.ns = */ "veh";
    veh.action = /* path_points.action = */ visualization_msgs::Marker::ADD;

    veh.id = 0;
    // path_points.id = 1;
    veh.type = visualization_msgs::Marker::CUBE;
    // path_points.type = visualization_msgs::Marker::POINTS;

    veh.scale.x = 1.0;
    veh.scale.y = 1.0;
    veh.scale.z = 1.0;
    veh.color.r = 0.0f;
    veh.color.g = 1.0f;
    veh.color.b = 0.0f;
    veh.color.a = 1.0f;
    veh.lifetime = ros::Duration();

    // path_points.scale.x = 0.2;
    // path_points.scale.y = 0.2;
    // path_points.color.g = 1.0f;
    // path_points.color.a = 1.0;
    // path_points.pose.orientation.w = 1.0;

    // for (const geometry_msgs::Point p : path) {
    //   path_points.points.push_back(p);
    // }

    while (pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    ROS_DEBUG_ONCE("Subscriber detected");
    veh.pose.position = pos;
    veh.pose.orientation = ori;

    // ROS_INFO("%lui",sizeof(path_points.points));

    pub.publish(veh);
    // pub.publish(path_points);

    // ros::spinOnce();
    r.sleep();
  }
}
