#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "rosgraph_msgs/Clock.h"

#include "chrono_ros_com/MessageHandler.h"
#include "chrono_ros_com/LidarPublisher.h"

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42

MessageHandler handler;

ros::Publisher gps_pub;
ros::Publisher clock_pub;

ros::Subscriber key_sub;

Quaternion q;

void keyboardCallback(const std_msgs::Int8::ConstPtr &msg) {
  switch (msg->data) {
  case KEYCODE_R:
    handler.TurnRight();
    break;
  case KEYCODE_L:
    handler.TurnLeft();
    break;
  case KEYCODE_U:
    handler.IncreaseThrottle();
    break;
  case KEYCODE_D:
    handler.DecreaseThrottle();
    break;
  }
}

void publishPose() {
  geometry_msgs::Point p;
  p.x = handler.Pose().x;
  p.y = handler.Pose().y;
  p.z = handler.Pose().z;
  geometry_msgs::Quaternion q;
  q.x = handler.Orientation().x;
  q.y = handler.Orientation().y;
  q.z = handler.Orientation().z;
  q.w = -handler.Orientation().w;

  geometry_msgs::Pose pose;
  pose.position = p;
  pose.orientation = q;

  gps_pub.publish(pose);
}

void publishTime() {
  ros::Time t(ros::Time(handler.Time()));
  rosgraph_msgs::Clock time;
  time.clock = t;
  clock_pub.publish(time);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "chrono_ros_com");
  ros::NodeHandle n;

  gps_pub = n.advertise<geometry_msgs::Pose>("gps_msgs", 1000);
  clock_pub = n.advertise<rosgraph_msgs::Clock>("clock", 1000);

  key_sub = n.subscribe<std_msgs::Int8>("key_msgs", 1000, keyboardCallback);


  LidarPublisher lidar_pub(argc, argv, handler);

  try {
    while (ros::ok()) {
      handler.ReceiveAndHandle();

      publishPose();

      publishTime();

      // lidar_pub.Publish(handler.LidarData());
      lidar_pub.PublishPointCloud(handler.LidarData());

      ros::spinOnce();
    }
  } catch (std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  ros::shutdown();
}
