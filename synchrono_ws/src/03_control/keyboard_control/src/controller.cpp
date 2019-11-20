#include "keyboard_control/controller.h"

struct termios cooked, raw;
int kfd = 0;

Controller::Controller(ros::NodeHandle &n) {
  pub_ = n.advertise<common_msgs::Control>("control", 1000);
  signal(SIGINT, quit);
};

void Controller::keyLoop() {
  char c;
  bool dirty = false;
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  ROS_INFO("Reading from keyboard");
  ROS_INFO("----------------------------");
  ROS_INFO("Use arrow keys to print something out");

  while (ros::ok()) {
    if (read(kfd, &c, 1) < 0) {
      ROS_ERROR("Failed to read.");
      break;
    }

    switch (c) {
    case KEYCODE_R:
      ROS_DEBUG("LEFT");
      msg_.steering -= .01;
      dirty = true;
      break;
    case KEYCODE_L:
      ROS_DEBUG("RIGHT");
      msg_.steering += .01;
      dirty = true;
      break;
    case KEYCODE_U:
      ROS_DEBUG("UP");
      msg_.throttle += .01;
      msg_.braking -= .025;
      dirty = true;
      break;
    case KEYCODE_D:
      ROS_DEBUG("DOWN");
      msg_.braking += .01;
      msg_.throttle -= .025;
      dirty = true;
      break;
    }

    if (dirty == true) {
      clamp();
      pub_.publish(msg_);
      dirty = false;
    }
  }
}

void Controller::clamp() {
  msg_.throttle = msg_.throttle > 1 ? 1 : msg_.throttle < 0 ? 0 : msg_.throttle;
  msg_.steering =
      msg_.steering > 1 ? 1 : msg_.steering < -1 ? -1 : msg_.steering;
  msg_.braking = msg_.braking > 1 ? 1 : msg_.braking < 0 ? 0 : msg_.braking;
}

void Controller::quit(int sig) {
  ros::shutdown();
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}
