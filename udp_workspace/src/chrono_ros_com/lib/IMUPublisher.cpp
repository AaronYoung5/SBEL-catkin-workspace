#include "chrono_ros_com/IMUPublisher.h"

IMUPublisher::IMUPublisher(ros::NodeHandle n, MessageHandler &handler) : m_handler(handler), m_n(n) {
  m_pub = m_n.advertise<sensor_msgs::Imu>("imu", 1000);

  m_imu.header.frame_id = "imu";
}

void IMUPublisher::PublishIMU() {
  IMU data = m_handler.IMUData();

  m_imu.header.stamp = ros::Time::now();

  m_imu.linear_acceleration.x = data.linear_acceleration.x;
  m_imu.linear_acceleration.y = data.linear_acceleration.y;
  m_imu.linear_acceleration.z = data.linear_acceleration.z;

  m_imu.angular_velocity.x = data.angular_velocity.x;
  m_imu.angular_velocity.y = data.angular_velocity.y;
  m_imu.angular_velocity.z = data.angular_velocity.z;

  m_pub.publish(m_imu);
}
