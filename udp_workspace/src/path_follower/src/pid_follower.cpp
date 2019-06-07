// ROS includes
#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float32.h"
#include <chrono_ros_interface/Cones.h>
#include <chrono_ros_interface/SensorStructs.h>
#include <tf/transform_datatypes.h>

// Package specific includes
#include "path_follower/pid_follower.h"

Location init_loc;
Vector3D rel_pos;
Vector3D apos, ipos, lastPos;
Quaternion orientation;

std::vector<Vector3D> center_line;

ros::Publisher steering_pub;

bool loc_set = false;
bool pos_set = false;

float distance(Vector3D v1, Vector3D v2) {
  return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2) + pow(v1.z - v2.y, 2));
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
  if (!loc_set) {
    init_loc = Location::NavSatFix_To_Location(msg);
    loc_set = true;
  } else {
    rel_pos.x = init_loc.altitude - msg->altitude;
    rel_pos.y = init_loc.longitude - msg->longitude;
    rel_pos.z = init_loc.latitude - msg->latitude;
  }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  tf::Quaternion q = tf::createQuaternionFromRPY(msg->angular_velocity.x,
                                                 msg->angular_velocity.y,
                                                 msg->angular_velocity.z);
  orientation.x = q.x();
  orientation.y = q.y();
  orientation.z = q.z();
  orientation.w = q.w();
}

void vehCallback(const geometry_msgs::Point::ConstPtr &msg) {
  if (!pos_set) {
    ipos.x = msg->x;
    ipos.y = msg->y;
    ipos.z = msg->z;
    lastPos = ipos;
    pos_set = true;
  }
  lastPos = apos;
  apos.x = ipos.x - msg->x;
  apos.y = ipos.y - msg->y;
  apos.z = ipos.z - msg->z;
}

void coneCallback(const chrono_ros_interface::Cones::ConstPtr &msg) {
  if (init_loc.altitude == 0 || (orientation.x == 0 && orientation.y == 0 &&
                                 orientation.z == 0 && orientation.w == 0)) {
    return;
  }

  int num = sizeof(msg->blue_cones);

  if (center_line.size() == 0) {
    for (int i = 0; i < num; i++) {
      center_line.push_back(
          Vector3D{(float)(msg->blue_cones[i].pose.position.x +
                           msg->yellow_cones[i].pose.position.x) /
                       2,
                   (float)(msg->blue_cones[i].pose.position.y +
                           msg->yellow_cones[i].pose.position.y) /
                       2,
                   (float)(msg->blue_cones[i].pose.position.z +
                           msg->yellow_cones[i].pose.position.z) /
                       2});
    }
  }

  int seekIndex = 0;
  float closestDistance = 1000;
  for (int i = 0; i < num; i++) {
    float d = distance(center_line[i], apos);
    if (d < closestDistance) {
      closestDistance = d;
      seekIndex = i;
    }
  }
  float steering =
      PIDFollower::Steering(center_line, seekIndex, apos, orientation);
  std_msgs::Float32 s;
  s.data = steering;
  steering_pub.publish(s);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pid_follower");
  ros::NodeHandle n;

  ros::Subscriber cone_sub =
      n.subscribe<chrono_ros_interface::Cones>("cones", 10, coneCallback);
  ros::Subscriber veh_sub =
      n.subscribe<geometry_msgs::Point>("veh", 10, vehCallback);
  ros::Subscriber gps_sub =
      n.subscribe<sensor_msgs::NavSatFix>("gps", 10, gpsCallback);
  ros::Subscriber imu_sub =
      n.subscribe<sensor_msgs::Imu>("imu", 10, imuCallback);

  steering_pub = n.advertise<std_msgs::Float32>("steering", 10);

  ros::spin();
}

float PIDFollower::Steering(std::vector<Vector3D> path, int seekIndex,
                            Vector3D pos, Quaternion orienation) {
  Vector3D v1 = pos - lastPos;
  Vector3D v2 = path[1] - pos;

  float angle = v1.dot(v2) / (Vector3D::Length(v1) * Vector3D::Length(v2));

  std::cout << angle << std::endl;

  if (angle < 0) {
    return -1;
  } else if (angle > 0) {
    return 1;
  }
  return 0;
  //
  // float len = Vector3D::Length(v);
  // std::cout << "(" << v.x << ", " << v.y << ", " << v.z << ")"
  //           << std::endl;
  // return len;
  // return -1;
}
