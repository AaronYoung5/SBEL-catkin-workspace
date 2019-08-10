#include "cone_bounce_control/controller.h"

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

  float dist_off_screen = 1000;
  float dist_from_top = height / 2.0f;
  float dist_from_left = height / 2.0f;

  Triangle<> tri(Vec2<>(-dist_off_screen, height),
                 Vec2<>(width + dist_off_screen, height),
                 Vec2<>(dist_from_left, dist_from_top));

  if (green_cones.size() == 0 && red_cones.size() == 0) {
    control.throttle = 0;
    control.steering = 0;
  } else {
    for (opencv_msgs::Cone cone : green_cones) {
      Vec2<> avg((cone.tl.x + cone.br.x) / 2.0f,
                 (cone.tl.y + cone.br.y) / 2.0f);
      if (Triangle<>::IsInside(avg, tri)) {
        control.steering += avg.x() < (width / 2.0f)
                                ? 0.6f
                                : avg.x() < (width * .75f) ? 0.3f : 0.15f;
      }
    }
    for (opencv_msgs::Cone cone : red_cones) {
      Vec2<> avg((cone.tl.x + cone.br.x) / 2.0f,
                 (cone.tl.y + cone.br.y) / 2.0f);
      if (Triangle<>::IsInside(avg, tri)) {
        control.steering -= avg.x() > (width / 2.0f)
                                ? 0.6f
                                : avg.x() > (width * .75f) ? 0.3f : 0.15f;
      }
    }
    control.throttle = .12;
  }

  clamp(control);
  pub_.publish(control);
  // if (msg->green_cones)
}

void Controller::clamp(common_msgs::Control &control) {
  control.throttle =
      control.throttle > 1 ? 1 : control.throttle < 0 ? 0 : control.throttle;
  control.steering =
      control.steering > 1 ? 1 : control.steering < -1 ? -1 : control.steering;
}
