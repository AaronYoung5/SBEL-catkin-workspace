#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <chrono_ros_interface/Cones.h>

ros::Subscriber cones_sub;
ros::Publisher markers_pub;

void ConesToMarkers(const chrono_ros_interface::Cones::ConstPtr &msg) {

  int num = msg->size.data;
  visualization_msgs::MarkerArray markers;
  markers.markers.resize(num * 2);

  int j = num * 2 - 1;
  for (int i = 0; i < num; i++, j--) {

    markers.markers[i].header.stamp = markers.markers[j].header.stamp =
        ros::Time::now();
    markers.markers[i].header.frame_id = markers.markers[j].header.frame_id =
        "test";

    markers.markers[i].ns = markers.markers[j].ns = i;
    markers.markers[i].id = 0;
    markers.markers[j].id = 1;
    markers.markers[i].type = markers.markers[j].type =
        visualization_msgs::Marker::CYLINDER;
    markers.markers[i].action = markers.markers[j].action =
        visualization_msgs::Marker::ADD;

    markers.markers[i].pose.position.x = msg->blue_cones[i].pose.position.x;
    markers.markers[i].pose.position.y = msg->blue_cones[i].pose.position.y;
    markers.markers[i].pose.position.z = msg->blue_cones[i].pose.position.z;
    markers.markers[j].pose.position.x = msg->yellow_cones[i].pose.position.x;
    markers.markers[j].pose.position.y = msg->yellow_cones[i].pose.position.y;
    markers.markers[j].pose.position.z = msg->yellow_cones[i].pose.position.z;

    markers.markers[i].pose.orientation.w = 1;
    markers.markers[j].pose.orientation.w = 1;

    // std::cout << blue_cone.pose.position.x << std::endl;

    markers.markers[i].scale.x = 1;
    markers.markers[i].scale.y = 1;
    markers.markers[i].scale.z = 3;
    markers.markers[j].scale.x = 1;
    markers.markers[j].scale.y = 1;
    markers.markers[j].scale.z = 3;

    markers.markers[i].color.r = 0.0f;
    markers.markers[i].color.b = 1.0f;
    markers.markers[i].color.g = 0.0f;
    markers.markers[i].color.a = 1.0f;
    markers.markers[j].color.r = 1.0f;
    markers.markers[j].color.b = 0.0f;
    markers.markers[j].color.g = 1.0f;
    markers.markers[j].color.a = 1.0f;

    markers.markers[i].lifetime = markers.markers[j].lifetime = ros::Duration();
  }
  markers_pub.publish(markers);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cones_to_markers");
  ros::NodeHandle n;

  cones_sub =
      n.subscribe<chrono_ros_interface::Cones>("cones", 1000, ConesToMarkers);

  markers_pub = n.advertise<visualization_msgs::MarkerArray>("markers", 1000);

  ros::spin();
}
