#include "cone_midpoint_control/controller.h"

Controller::Controller(ros::NodeHandle &n) {
  std::string control_topic, cone_topic;
  n.param<std::string>("control_topic", control_topic, "control");
  n.param<std::string>("cone_topic", cone_topic, "cone_image_map");

  pub_ = n.advertise<common_msgs::Control>(control_topic, 1);
  sub_ = n.subscribe(cone_topic, 1, &Controller::imageCallback, this);
}

void Controller::imageCallback(const opencv_msgs::ConeImageMap::ConstPtr &msg) {
  common_msgs::Control control;
  int height = msg->height;
  int width = msg->width;

  std::vector<opencv_msgs::Cone> green_cones = msg->green_cones;
  std::vector<opencv_msgs::Cone> red_cones = msg->red_cones;

  if (green_cones.size() == 0 && red_cones.size() == 0) {
    control.throttle = 0;
    control.steering = 0;
    control.braking = 1;
  } else if (green_cones.size() == 0 || red_cones.size() == 0) {
    control.throttle = .15;
    control.steering = green_cones.size() == 0 ? .6 : -.6;
  } else {
    Vec2<> avgG((green_cones[0].tl.x + green_cones[0].br.x) / 2,
                (green_cones[0].tl.y + green_cones[0].br.y) / 2);
    for (int i = 1; i < green_cones.size(); i++) {
      Vec2<> holdG((green_cones[i].tl.x + green_cones[i].br.x) / 2,
                   (green_cones[i].tl.y + green_cones[i].br.y) / 2);
      if (holdG.y() > avgG.y()) {
        avgG = holdG;
      }
    }
    Vec2<> avgR((red_cones[0].tl.x + red_cones[0].br.x) / 2,
                (red_cones[0].tl.y + red_cones[0].br.y) / 2);
    for (int i = 1; i < red_cones.size(); i++) {
      Vec2<> holdR((red_cones[i].tl.x + red_cones[i].br.x) / 2,
                   (red_cones[i].tl.y + red_cones[i].br.y) / 2);
      if (holdR.y() > avgR.y()) {
        avgR = holdR;
      }
    }
    Vec2<> destination((avgR + avgG) / 2);
    Vec2<> center(width / 2, height / 2);
    control.steering = (destination.x() - center.x()) / center.x();
    control.throttle = .15;
  }

  clamp(control);
  pub_.publish(control);
}
