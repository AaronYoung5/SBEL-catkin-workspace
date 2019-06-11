#include "ros/ros.h"
#include "sensor_msgs::Imu.h"
#include "geometry_msgs/Twist.h"

class VelocityEstimator {
private:
  ros::Publisher pub_;
  ros::Subscriber imu_sub_;

public:
  VelocityEstimator();
  void spin();
private:
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
};

VelocityEstimator::VelocityEstimator(ros::NodeHande n)
    : pub_(n.advertise<geometry_msgs::Twist>("velocity_estimates", 10)),
      imu_sub_(n.subscribe<sensor_msgs::Imu>(
          "imu", 10, &VelocityEstimator::imuCallback, this)) {}

void VelocityEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "velocity_estimator");
  ros::NodeHandle n;

  VelocityEstimator velocityEstimator;

  while(ros::ok()) {
    velocityEstimator.spin();
  }
}
