// ROS includes
#include "ros/ros.h"
#include "std_msgs/Int8.h"

// Definition declarations
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71


ChronoRosVehicle chrono_ros_vehicle;

void keyboardCallback(const std_msgs::Int8::ConstPtr &msg) {
  switch (msg->data) {
  case KEYCODE_R:
    chrono_ros_vehicle.turnRight();
    break;
  case KEYCODE_L:
    chrono_ros_vehicle.turnLeft();
    break;
  case KEYCODE_U:
    chrono_ros_vehicle.increaseThrottle();
    break;
  case KEYCODE_D:
    chrono_ros_vehicle.decreaseThrottle();
    break;
  }
}

int main(int argc, char **argv) {
  ros::init(argv, argv, "chrono_ros");
  ros::NodeHandle n;

  keyboard_sub =
      n.subscribe<std_msgs::Int8>("keyboard_msgs", 1000, keyboardCallback);
}
