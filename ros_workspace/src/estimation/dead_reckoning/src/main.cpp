// ROS include
#include "ros/ros.h"

// Message type includes
#include "common_msgs/VehState.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "rosgraph_msgs/Clock.h"
#include "sensor_msgs/Imu.h"

// Include utilities
#include "common_utilities/Vector.h"

using namespace common_utilities;

class Orientation {
private:
  ros::Publisher pub_;

  ros::Subscriber imu_sub_;
  ros::Subscriber clock_sub_;

  float time_, dt_;

  common_msgs::VehState state_;

  Vector3D<> velocity_;

public:
  Orientation(ros::NodeHandle &n)
      : pub_(n.advertise<common_msgs::VehState>("vehicle", 10)),
        imu_sub_(n.subscribe<sensor_msgs::Imu>(
            "imu", 10, &Orientation::imuCallback, this)),
        clock_sub_(n.subscribe<rosgraph_msgs::Clock>(
            "clock", 10, &Orientation::clockCallback, this)),
        time_(0), dt_(0), velocity_(0, 0, 0) {}

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    // std::cout << "Delta T :: " << dt_ << std::endl;

    velocity_ = velocity_ + Vector3D<>(msg->linear_acceleration.x,
                                       msg->linear_acceleration.y,
                                       msg->linear_acceleration.z);

    // velocity_.x += msg->linear_acceleration.x;
    // velocity_.y += msg->linear_acceleration.y;
    // velocity_.z += msg->linear_acceleration.z;

    state_.state.position.x += velocity_.x();
    state_.state.position.y += velocity_.y();
    state_.state.position.z += velocity_.z();

    std::cout << "Position :: "
              << "(" << state_.state.position.x << ", "
              << state_.state.position.y << ", " << state_.state.position.z
              << ")" << std::endl;

    pub_.publish(state_);

    dt_ = 0;
  }

  void clockCallback(const rosgraph_msgs::Clock::ConstPtr &msg) {
    float temp = msg->clock.sec + msg->clock.nsec / 1e9;

    // std::cout << "Time :: " << temp << std::endl;

    dt_ += temp - time_;
    time_ = temp;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "dead_reckoning");
  ros::NodeHandle n;

  Orientation orientation(n);

  ros::spin();
}
