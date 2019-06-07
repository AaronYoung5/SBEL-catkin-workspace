#include "chrono_ros_com/ConePublisher.h"

ConePublisher::ConePublisher(ros::NodeHandle n, MessageHandler &handler) : m_handler(handler), m_n(n) {
  m_pub = m_n.advertise<geometry_msgs::PoseArray>("cones", 1000);

  m_cones.header.frame_id = "cones";
}

void ConePublisher::PublishCones() {
  std::vector<Position> blue_cones = m_handler.BlueCones();
  std::vector<Position> yellow_cones = m_handler.YellowCones();

  m_cones.header.stamp = ros::Time::now();

  m_cones.poses.resize(blue_cones.size() * 2);
  for (int i = 0; i < blue_cones.size(); i++) {
    geometry_msgs::Pose bp; geometry_msgs::Pose yp;

    bp.position.x = blue_cones[i].x;
    bp.position.y = blue_cones[i].y;
    bp.position.z = blue_cones[i].z;
    yp.position.x = yellow_cones[i].x;
    yp.position.y = yellow_cones[i].y;
    yp.position.z = yellow_cones[i].z;

    // std::cout << "(" << yp.position.x << ", " << yp.position.y << ", " << yp.position.z << ")" <<
    // std::endl;

    m_cones.poses[i] = bp;
    m_cones.poses[blue_cones.size() * 2 - i - 1] = yp;
  }

  m_pub.publish(m_cones);
}
