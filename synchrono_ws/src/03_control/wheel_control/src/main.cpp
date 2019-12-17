#include "common_msgs/Control.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class SteeringWheel {
public:
  SteeringWheel(ros::NodeHandle &n);

private:
  ros::Publisher pub;
  ros::Subscriber sub;

  void clamp();
  float map(float x, float in_min, float in_max, float out_min, float out_max);
  void steeringCallback(const sensor_msgs::Joy::ConstPtr &msg);

  float throttle_;
  float steering_;
  float braking_ = 0;

  bool throttle_updated_, steering_updated_, braking_updated_;
};

SteeringWheel::SteeringWheel(ros::NodeHandle &n)
    : throttle_(-1), steering_(0), braking_(-1), throttle_updated_(false),
      braking_updated_(false), steering_updated_(false) {
  pub = n.advertise<common_msgs::Control>("control", 1000);
  sub = n.subscribe("joy", 1000, &SteeringWheel::steeringCallback, this);
};

void SteeringWheel::steeringCallback(const sensor_msgs::Joy::ConstPtr &msg) {
  if (!throttle_updated_ && msg->axes[2] == 0) {
    throttle_ = 0;
  } else {
    throttle_updated_ = true;
    throttle_ = map(msg->axes[2], -1, 1, 0, 1);
  }
  //
  // std::cout << "Braking :: " << msg->axes[3] << std::endl;

  // if (!braking_updated_ && msg->axes[3] == 0) {
  // braking_ = 0;
  // } else {
  // braking_updated_ = true;
  braking_ = map(msg->axes[3], -1, 1, 0, 1);
  // }

  // if (!steering_updated_ && msg->axes[0] == 0) {
  // steering_ = 0;
  // } else {
  // steering_updated_ = true;
  steering_ = msg->axes[0];
  // }

  common_msgs::Control control;
  control.throttle = throttle_;
  control.steering = steering_;
  control.braking = braking_;

  pub.publish(control);
}

void SteeringWheel::clamp() {
  throttle_ = throttle_ > 1 ? 1 : throttle_ < -1 ? -1 : throttle_;
  steering_ = steering_ > 1 ? 1 : steering_ < -1 ? -1 : steering_;
}

float SteeringWheel::map(float x, float in_min, float in_max, float out_min,
                         float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wheel_control");
  ros::NodeHandle n;

  SteeringWheel SteeringWheel(n);

  ros::spin();
}
