// ROS include
#include "ros/ros.h"

// Message type includes
#include "common_msgs/VehState.h"
#include "visualization_msgs/Marker.h"

// Utilities
#include "common_utilities/Vector.h"

using namespace common_utilities;

class CarVisualizer {
private:
  ros::Subscriber sub_;

  ros::Publisher pub_;

public:
  CarVisualizer(ros::NodeHandle &n)
      : sub_(n.subscribe<common_msgs::VehState>(
            "vehicle", 10, &CarVisualizer::callback, this)),
        pub_(n.advertise<visualization_msgs::Marker>("car_rviz", 10)) {}

  void callback(const common_msgs::VehState::ConstPtr &msg) {
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "chrono_rviz";

    marker.ns = marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = msg->state.position.x;
    marker.pose.position.y = msg->state.position.y;
    marker.pose.position.z = msg->state.position.z;

    marker.pose.orientation.w = 1;

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    marker.color.r = 0.0f;
    marker.color.b = 0.0f;
    marker.color.g = 1.0f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration();

    if (pub_.getNumSubscribers() == 0) {
      ROS_WARN_ONCE("No subcribers for the car_rviz topic");
    } else {
      pub_.publish(marker);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_rviz");
  ros::NodeHandle n;

  CarVisualizer visualizer(n);

  ros::spin();
}