// ROS includes
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/ros.h"
#include "std_msgs/Int8.h"

// Chrono/ROS includes
#include "chrono_ros/chrono_ros_path_follower_launcher.h"

#include <signal.h>

// Definition declarations
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
// #define KEYCODE_Q 0x71

ChronoRosPathFollowerLauncher test;
// ros::Publisher gps_pub;

// void keyboardCallback(const std_msgs::Int8::ConstPtr &msg) {
//   // switch (msg->data) {
//   // case KEYCODE_R:
//   //   chrono_ros_path_follower_launcher.TurnRight();
//   //   break;
//   // case KEYCODE_L:
//   //   chrono_ros_path_follower_launcher.TurnLeft();
//   //   break;
//   // case KEYCODE_U:
//   //   chrono_ros_path_follower_launcher.IncreaseThrottle();
//   //   break;
//   // case KEYCODE_D:
//   //   chrono_ros_path_follower_launcher.DecreaseThrottle();
//   //   break;
//   // }
// }
//
// void publishPose() {
//   geometry_msgs::Point p;
//   p.x = -chrono_ros_path_follower_launcher.GetVehicle()
//              .GetVehicle()
//              .GetVehicleCOMPos()
//              .x();
//   p.y = chrono_ros_path_follower_launcher.GetVehicle()
//             .GetVehicle()
//             .GetVehicleCOMPos()
//             .y();
//   p.z = chrono_ros_path_follower_launcher.GetVehicle()
//             .GetVehicle()
//             .GetVehicleCOMPos()
//             .z();
//   geometry_msgs::Quaternion q;
//   q.x = chrono_ros_path_follower_launcher.GetVehicle()
//             .GetVehicle()
//             .GetVehicleRot()
//             .e1();
//   q.y = chrono_ros_path_follower_launcher.GetVehicle()
//             .GetVehicle()
//             .GetVehicleRot()
//             .e2();
//   q.z = chrono_ros_path_follower_launcher.GetVehicle()
//             .GetVehicle()
//             .GetVehicleRot()
//             .e3();
//   q.w = -chrono_ros_path_follower_launcher.GetVehicle()
//              .GetVehicle()
//              .GetVehicleRot()
//              .e0();
//   geometry_msgs::Pose pose;
//   pose.position = p;
//   pose.orientation = q;
//
//   gps_pub.publish(pose);
// }

void quit(int sig) {
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "chrono_ros_path_follower");
  ros::NodeHandle n;

  // ros::Subscriber keyboard_sub =
  //     n.subscribe<std_msgs::Int8>("key_msgs", 1000, keyboardCallback);

  // ros::Subscriber throttle_sub =
  //     n.subscribe<std_msgs::Int8>("throttle_msgs", 1000, keyboardCallback);

  //gps_pub = n.advertise<geometry_msgs::Pose>("gps_msgs", 1000);

  signal(SIGINT, quit);

  test.GetApp();

  while (ros::ok()) {
    // chrono_ros_path_follower_launcher.ChronoRun();

    // publishPose();

    quit(0);
    ros::spinOnce();
  }
}
