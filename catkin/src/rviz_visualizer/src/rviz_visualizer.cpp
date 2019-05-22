#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "rviz_visualizer/path_msg.h"
#include "std_msgs/Float32MultiArray.h"
#include "visualization_msgs/Marker.h"

std::vector<geometry_msgs::Point> path;

float pos[3];
bool pos_found = false;
bool path_recieved = false;

void draw_path(const rviz_visualizer::path_msg::ConstPtr &msg) {
  path_recieved = true;
  // path = msg;

  ROS_INFO_ONCE("PATH RECIEVED");

  int i = 0;
  for (const geometry_msgs::Point p : msg->points) {
    path.push_back(p);
  }
}

void gps_update(const std_msgs::Float32MultiArray::ConstPtr &array) {
  pos_found = true;
  ROS_INFO_ONCE("GPS RECIEVED");
  int i = 0;
  for (std::vector<float>::const_iterator it = array->data.begin();
       it != array->data.end(); ++it) {
    pos[i] = *it;
    i++;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rviz_visualizer");
  ros::NodeHandle n;

  ros::Subscriber path_sub =
      n.subscribe<rviz_visualizer::path_msg>("path_msg", 1, draw_path);
  ros::Subscriber gps_sub =
      n.subscribe<std_msgs::Float32MultiArray>("veh_gps", 1000, gps_update);

  ros::Publisher pub =
      n.advertise<visualization_msgs::Marker>("rviz_visualizer", 1000);

  ros::Rate r(30);

  while (ros::ok()) {
    ros::spinOnce();
    if (!(pos_found && path_recieved)) {
      ROS_WARN_ONCE("Please provide a car position and path");
      continue;
    }

    visualization_msgs::Marker veh, path_points;

    veh.header.frame_id = path_points.header.frame_id = "/rv_frame";
    veh.header.stamp = path_points.header.stamp = ros::Time::now();
    veh.ns = path_points.ns = "veh_and_path";
    veh.action = path_points.action = visualization_msgs::Marker::ADD;

    veh.id = 0;
    path_points.id = 1;
    veh.type = visualization_msgs::Marker::CUBE;
    path_points.type = visualization_msgs::Marker::POINTS;

    veh.pose.orientation.x = 0.0;
    veh.pose.orientation.y = 0.0;
    veh.pose.orientation.z = 0.0;
    veh.pose.orientation.w = 1.0;
    veh.scale.x = 1.0;
    veh.scale.y = 1.0;
    veh.scale.z = 1.0;
    veh.color.r = 0.0f;
    veh.color.g = 1.0f;
    veh.color.b = 0.0f;
    veh.color.a = 1.0f;
    veh.lifetime = ros::Duration();

    path_points.scale.x = 0.2;
    path_points.scale.y = 0.2;
    path_points.color.g = 1.0f;
    path_points.color.a = 1.0;
    path_points.pose.orientation.w = 1.0;

    for (const geometry_msgs::Point p : path) {
      path_points.points.push_back(p);
    }

    while (pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    ROS_DEBUG_ONCE("Subscriber detected");
    veh.pose.position.x = pos[0];
    veh.pose.position.y = pos[1];
    veh.pose.position.z = pos[2];

    // ROS_INFO("%lui",sizeof(path_points.points));

    pub.publish(veh);
    pub.publish(path_points);

    // ros::spinOnce();
    r.sleep();
  }
}
