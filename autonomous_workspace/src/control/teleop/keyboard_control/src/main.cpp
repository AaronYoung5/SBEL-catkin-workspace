#include "common_msgs/Control.h"
#include "ros/ros.h"

#include <signal.h>
#include <stdio.h>
#include <termios.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class Teleop {
public:
  Teleop();
  void keyLoop();
  void clamp();

private:
  ros::NodeHandle n;
  ros::Publisher pub;

  float throttle_;
  float steering_;
};

Teleop::Teleop() : throttle_(0), steering_(0) {
  pub = n.advertise<common_msgs::Control>("control", 1000);
};

int kfd = 0;
struct termios cooked, raw;

void Teleop::keyLoop() {
  char c;
  bool dirty = false;
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("----------------------------");
  puts("Use arrow keys to print something out");

  for (;;) {
    if (read(kfd, &c, 1) < 0) {
      perror("read():");
      exit(-1);
    }

    switch (c) {
    case KEYCODE_L:
      steering_ -= .01;
      // ROS_INFO("LEFT");
      dirty = true;
      break;
    case KEYCODE_R:
      steering_ += .01;
      // ROS_INFO("RIGHT");
      dirty = true;
      break;
    case KEYCODE_U:
      throttle_ += .01;
      // ROS_INFO("UP");
      dirty = true;
      break;
    case KEYCODE_D:
      // throttle_ -= .025;
      throttle_ -= .01;
      // ROS_INFO("DOWN");
      dirty = true;
      break;
    }

    clamp();

    if (dirty == true) {
      common_msgs::Control msg;
      msg.throttle = throttle_;
      msg.steering = steering_;
      pub.publish(msg);
      dirty = false;
    }
  }
}

void Teleop::clamp() {
  throttle_ = throttle_ > 1 ? 1 : throttle_ < -1 ? -1 : throttle_;
  steering_ = steering_ > 1 ? 1 : steering_ < -1 ? -1 : steering_;
}

void quit(int sig) {
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "teleop");
  Teleop teleop;

  signal(SIGINT, quit);
  teleop.keyLoop();
}
