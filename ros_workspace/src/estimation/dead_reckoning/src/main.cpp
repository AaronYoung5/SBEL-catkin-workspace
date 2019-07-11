// ROS include
#include "ros/ros.h"

// Message type includes
#include "common_msgs/VehState.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "rosgraph_msgs/Clock.h"
#include "sensor_msgs/Imu.h"

// Include utilities
// #include "common_utilities/Vector.h"

// using namespace common_utilities;

class Orientation {
private:
  ros::Publisher pub_;

  ros::Subscriber imu_sub_;
  ros::Subscriber clock_sub_;

  float time_, dt_;

public:
  Orientation(ros::NodeHandle &n)
      : pub_(n.advertise<common_msgs::VehState>("vehicle", 10)),
        imu_sub_(n.subscribe<sensor_msgs::Imu>(
            "imu", 10, &Orientation::imuCallback, this)),
        clock_sub_(n.subscribe<rosgraph_msgs::Clock>(
            "clock", 10, &Orientation::clockCallback, this)),
        time_(0), dt_(0) {}

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    common_msgs::VehState state;

    state.state.position.x = msg->linear_acceleration.x * dt_ * dt_;
    state.state.position.y = msg->linear_acceleration.y * dt_ * dt_;
    state.state.position.z = msg->linear_acceleration.z * dt_ * dt_;

    pub_.publish(state);

    dt_ = 0;
  }

  void clockCallback(const rosgraph_msgs::Clock::ConstPtr &msg) {
    float temp = msg->clock.sec + msg->clock.nsec / 10;
    dt_ += temp;
    time_ = temp;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "dead_reckoning");
  ros::NodeHandle n;

  Orientation orientation(n);

  ros::spin();
}
