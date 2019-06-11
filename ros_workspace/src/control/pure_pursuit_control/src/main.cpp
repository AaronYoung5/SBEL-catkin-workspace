#include "common_msgs/ConeMap.h"
#include "common_msgs/Control.h"
#include "common_msgs/VehState.h"
#include "ros/ros.h"

class PurePursuit {
private:
  ros::Publisher pub_;

  ros::Subscriber cone_sub_;
  ros::Subscriber state_sub_;

  common_msgs::ConeMap cone_map_;
public:
  PurePursuit(ros::NodeHandle n);
  void spin();
  void coneCallback(const common_msgs::ConeMap::ConstPtr &msg);
  void stateCallback(const common_msgs::VehState::ConstPtr &msg);
};

PurePursuit::PurePursuit(ros::NodeHandle n)
    : pub_(n.advertise<common_msgs::Control>("control", 10)),
      cone_sub_(n.subscribe("cones", 10, &PurePursuit::coneCallback, this)),
      state_sub_(n.subscribe("state", 10, &PurePursuit::stateCallback, this)) {}

void PurePursuit::coneCallback(const common_msgs::ConeMap::ConstPtr &msg) {
  cone_map_ = *msg;
}

void PurePursuit::stateCallback(const common_msgs::VehState::ConstPtr &msg) {

}

void PurePursuit::spin() {
  common_msgs::Control control;
  control.throttle.data = 0;
  control.braking.data = .1;
  control.steering.data = 0;

  pub_.publish(control);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pure_pursuit_control");
  ros::NodeHandle n;

  PurePursuit purePursuit(n);

  while (ros::ok()) {
    purePursuit.spin();
  }
}
