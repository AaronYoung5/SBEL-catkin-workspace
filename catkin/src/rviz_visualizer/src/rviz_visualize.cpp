#include "msg/path_msg.msg"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "visualization_msgs/Marker.h"

std::vector<float> path;
float pos[3];
bool pos_found = false;
bool path_recieved = false;

void draw_path(const msg::path_msg::ConstPtr &msg) {
  path_recieved = true;
  for (std::vector<float>::const_iterator it = msg->data.begin();
       it != msg->data.end(); ++it) {
    path.push_back(*it);
  }
}

void gps_update(const std_msgs::Float32MultiArray::ConstPtr &array) {
  pos_found = true;
  int i = 0;
  for (std::vector<float>::const_iterator it = array->data.begin();
       it != array->data.end(); ++it) {
    pos[i] = *it;
    i++;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rviz_visualize");
  ros::NodeHandle n;

  ros::Subscriber path_sub =
      n.subscribe<msg::path_msg>("path_msg", 1, draw_path);
  ros::Subscriber gps_sub =
      n.subscribe<std_msgs::Float32MultiArray>("veh_gps", 1000, gps_update);

  ros::Publisher pub =
      n.advertise<visualization_msgs::Marker>("rviz_visualizer", 1);

  ros::Rate r(60);

  while (ros::ok()) {
    if (!(pos_found && path_recieved)) {
      continue;
    }

    visualization_marker::Marker veh, path_line;

    veh.header.frame_id = path_line.header.frame_id = "rviz_visualizer_frame";
    veh.header.stamp = path_line.header.stamp = ros::Time::now();
    veh.ns = "veh";
    path_line.ns = "path";
    veh.action = path_line.action = visualization::Marker::ADD;
    veh.pose.orientation.w = path_line.pose.orientation.w = 1.0;

    veh.id = 0;
    path_line.id = 0;
    veh.type = visualization_msgs::Marker::CUBE;
    path_line.type = visualization_msgs::Marker::LINE_STRIP;

    veh.pose.position.x = path[0];
    veh.pose.position.y = path[1];
    veh.pose.position.z = path[2];
    veh.pose.orientation.x = 0.0;
    veh.pose.orientation.y = 0.0;
    veh.pose.orientation.z = 0.0;
    veh.pose.orientation.q = 1.0;
    veh.scale.x = 1.0;
    veh.scale.y = 1.0;
    veh.scale.z = 1.0;
    veh.color.r = 0.0f;
    veh.color.g = 1.0f;
    veh.color.b = 0.0f;
    veh.color.a = 1.0f;
    veh.lifetime = ros::Duration();

    path_line.scale.x = 0.1f;
    path_line.color.b = 1.0f;
    path_line.a = 1.0f;
    path_line.points = path;

    pub.publish(path_line);
    pub.publish(veh);

    r.sleep();
  }
}
