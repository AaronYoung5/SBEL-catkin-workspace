// ROS include
#include "ros/ros.h"

// Message type includes
#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"

// Service type includes
#include "common_srvs/Path.h"

// Utilities
#include "common_utilities/Vector.h"

using namespace common_utilities;

class PathVisualizer {
private:
  ros::ServiceClient client_;

  ros::Publisher pub_;

  std::vector<Vector3D<>> path_;

public:
  PathVisualizer(ros::NodeHandle &n)
      : client_(n.serviceClient<common_srvs::Path>("path", true)),
        pub_(n.advertise<visualization_msgs::MarkerArray>("path_rviz", 10)) {
    common_srvs::Path path;
    while (!client_.call(path)) {
    }

    for (geometry_msgs::Point point : path.response.points) {
      path_.push_back(Vector3D<>(point.x, point.y, point.z));
    }
  }

  void publish() {
    int num = path_.size();

    visualization_msgs::MarkerArray markers;
    markers.markers.resize(num);

    for (int i = 0; i < num; i++) {

      markers.markers[i].header.stamp = ros::Time::now();
      markers.markers[i].header.frame_id = "chrono_rviz";

      markers.markers[i].ns = i;
      markers.markers[i].id = 0;
      markers.markers[i].type = visualization_msgs::Marker::SPHERE;
      markers.markers[i].action = visualization_msgs::Marker::ADD;

      markers.markers[i].pose.position.x = path_[i].x();
      markers.markers[i].pose.position.y = path_[i].y();
      markers.markers[i].pose.position.z = path_[i].z();

      markers.markers[i].pose.orientation.w = 1;

      markers.markers[i].scale.x = 1;
      markers.markers[i].scale.y = 1;
      markers.markers[i].scale.z = 1;

      markers.markers[i].color.r = 0.0f;
      markers.markers[i].color.b = 1.0f;
      markers.markers[i].color.g = 0.0f;
      markers.markers[i].color.a = 1.0f;

      markers.markers[i].lifetime = ros::Duration();
    }

    while (pub_.getNumSubscribers() == 0) {
      ROS_WARN_ONCE("No subcribers for the path_rviz topic");
      ros::spinOnce();
    }

    ROS_INFO("Publishing path_rviz ...");

    pub_.publish(markers);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_rviz");
  ros::NodeHandle n;

  PathVisualizer visualizer(n);

  for (int i = 0; i < 5; i++) {
    visualizer.publish();
  }
}
