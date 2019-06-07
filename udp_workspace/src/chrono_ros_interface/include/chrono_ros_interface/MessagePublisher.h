// ROS includes
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "rosgraph_msgs/Clock.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"

// Package includes
#include "chrono_ros_interface/MessageHandler.h"
#include "chrono_ros_interface/Cones.h"


class MessagePublisher {
private:
  MessageHandler& m_handler;

  ros::Publisher m_lidar_pub;
  ros::Publisher m_imu_pub;
  ros::Publisher m_gps_pub;
  ros::Publisher m_time_pub;
  ros::Publisher m_cone_pub;
  ros::Publisher m_veh_pub;

  std::string m_frame_id;
public:
  MessagePublisher(ros::NodeHandle n, MessageHandler& handler);

  void Publish(PublisherCodes::Code message_to_publish);
  void PublishAll();

private:
  sensor_msgs::PointCloud2 ToPointCloud2(std::vector<Vector3D> data);
  sensor_msgs::NavSatFix ToNavSatFix(Location data);
  sensor_msgs::Imu ToImu(IMU data);
  chrono_ros_interface::Cones ToCones(std::vector<Vector3D> blue_cones, std::vector<Vector3D> yellow_cones);
};
