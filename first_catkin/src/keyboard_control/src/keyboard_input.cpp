#include "ros/ros.h"
#include "std_msgs/Int8.h"

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

private:
  ros::NodeHandle n;
  ros::Publisher pub;
};

Teleop::Teleop() { pub = n.advertise<std_msgs::Int8>("key_msgs", 1000); };

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
      // ROS_INFO("LEFT");
      dirty = true;
      break;
    case KEYCODE_R:
      // ROS_INFO("RIGHT");
      dirty = true;
      break;
    case KEYCODE_U:
      // ROS_INFO("UP");
      dirty = true;
      break;
    case KEYCODE_D:
      // ROS_INFO("DOWN");
      dirty = true;
      break;
    }

    if (dirty == true) {
      std_msgs::Int8 msg;
      msg.data = c;
      pub.publish(msg);
      dirty = false;
    }
  }
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
