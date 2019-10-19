#include <ros/ros.h>

#include <termios.h>
#include <signal.h>

#include <common_msgs/Control.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class Controller {
private:
  ros::Publisher pub_;

  common_msgs::Control msg_;

public:
  Controller(ros::NodeHandle &n);
  void keyLoop();
  void clamp();
  static void quit(int sig);
};
