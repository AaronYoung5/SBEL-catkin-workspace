// ROS include
#include "ros/ros.h"

// Message type includes
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"

class Orientation {
private:
  ros::Publisher pub_;

  ros::Subscriber sub_;

public:
  Orientation(ros::NodeHandle &n)
      : pub_(n.advertise<geometry_msgs::Quaternion>("vehicle/orientation", 10)),
        sub_(n.subscribe<sensor_msgs::Imu>("imu", 10, &Orientation::callback,
                                           this)) {}

  void callback(const sensor_msgs::Imu::ConstPtr &msg) {
    geometry_msgs::Quaternion q;

    
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "orientation");
  ros::NodeHandle n;

  Orientation orientation(n);

  ros::spin();
}
