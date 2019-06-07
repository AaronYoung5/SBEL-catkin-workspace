#include "chrono_ros_com/GPSPublisher.h"

GPSPublisher::GPSPublisher(ros::NodeHandle n, MessageHandler &handler) : m_handler(handler), m_n(n) {
  m_pub = m_n.advertise<sensor_msgs::NavSatFix>("gps", 1000);

  m_gps.header.frame_id = "gps";
  m_gps.status.status = 0;
  m_gps.status.service = 0;
}

void GPSPublisher::PublishGPS() {
  GPSPosition data = m_handler.GPSData();

  m_gps.header.stamp = ros::Time::now();

  m_gps.latitude = data.lat;
  m_gps.longitude = data.lon;
  m_gps.altitude = data.alt;

  m_gps.position_covariance[0] = 0.0;
  m_gps.position_covariance[4] = 0.0;
  m_gps.position_covariance[8] = 0.0;

  m_pub.publish(m_gps);
}
