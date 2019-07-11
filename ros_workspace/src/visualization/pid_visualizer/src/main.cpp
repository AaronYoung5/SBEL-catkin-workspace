// ROS include
#include "ros/ros.h"

// Message type includes
#include "common_msgs/PIDDebug.h"
#include "visualization_msgs/Marker.h"

// Utilities
#include "common_utilities/Vector.h"

using namespace common_utilities;

class PIDVisualizer {
private:
  ros::Subscriber sub_;

  ros::Publisher pub_;

public:
  PIDVisualizer(ros::NodeHandle &n)
      : sub_(n.subscribe<common_msgs::PIDDebug>(
            "pid_debug", 10, &PIDVisualizer::callback, this)),
        pub_(n.advertise<visualization_msgs::Marker>("pid_rviz", 10)) {}

  void callback(const common_msgs::PIDDebug::ConstPtr &msg) {
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "chrono_rviz";

    marker.ns = marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = msg->point_following.x;
    marker.pose.position.y = msg->point_following.y;
    marker.pose.position.z = msg->point_following.z;

    marker.pose.orientation.w = 1;

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    marker.color.r = 1.0f;
    marker.color.b = 0.0f;
    marker.color.g = 0.0f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration();

    if (pub_.getNumSubscribers() == 0) {
      ROS_WARN_ONCE("No subcribers for the pid_rviz topic");
    } else {
      pub_.publish(marker);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pid_rviz");
  ros::NodeHandle n;

  PIDVisualizer visualizer(n);

  ros::spin();
}
