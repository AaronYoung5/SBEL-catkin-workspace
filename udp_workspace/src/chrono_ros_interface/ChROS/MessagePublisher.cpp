#include "chrono_ros_interface/MessagePublisher.h"

MessagePublisher::MessagePublisher(ros::NodeHandle n, MessageHandler &handler)
    : m_handler(handler), m_frame_id("test") {
  m_lidar_pub = n.advertise<sensor_msgs::PointCloud2>("lidar", 1000);
  m_gps_pub = n.advertise<sensor_msgs::NavSatFix>("gps", 1000);
  m_imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1000);
  m_time_pub = n.advertise<rosgraph_msgs::Clock>("clock", 1000);
  m_cone_pub = n.advertise<chrono_ros_interface::Cones>("cones", 1000);
  m_veh_pub = n.advertise<geometry_msgs::Point>("veh", 1000);
}

void MessagePublisher::Publish(PublisherCodes::Code message_to_publish) {
  switch (message_to_publish) {
  case PublisherCodes::LIDAR: {
    if (m_handler.HasAllLidarData()) {
      std::vector<Vector3D> data = m_handler.LidarData();
      sensor_msgs::PointCloud2 cloud = ToPointCloud2(data);
      m_lidar_pub.publish(cloud);
    }
    break;
  }
  case PublisherCodes::GPS: {
    Location data = m_handler.GPSData();
    sensor_msgs::NavSatFix nsf = ToNavSatFix(data);
    m_gps_pub.publish(nsf);
    break;
  }
  case PublisherCodes::IMU: {
    IMU data = m_handler.IMUData();
    sensor_msgs::Imu imu = ToImu(data);
    m_imu_pub.publish(imu);
    break;
  }
  case PublisherCodes::VEHICLE: {
    Vector3D pos = m_handler.Position();
    geometry_msgs::Point p;
    p.x = pos.x;
    p.y = pos.y;
    p.z = pos.z;
    m_veh_pub.publish(p);
    break;
  }
  case PublisherCodes::TIME: {
    rosgraph_msgs::Clock time;
    time.clock = ros::Time(m_handler.Time());
    m_time_pub.publish(time);
    break;
  }
  case PublisherCodes::CONE: {
    std::vector<Vector3D> blue_cones = m_handler.BlueCones();
    std::vector<Vector3D> yellow_cones = m_handler.YellowCones();
    chrono_ros_interface::Cones cones = ToCones(blue_cones, yellow_cones);
    m_cone_pub.publish(cones);

    // nav_msgs::Path yellow_cones
    break;
  }
  }
}

void MessagePublisher::PublishAll() {
  for (int i = PublisherCodes::LIDAR; i <= PublisherCodes::CONE; i++) {
    Publish((PublisherCodes::Code)i);
  }
}

sensor_msgs::PointCloud2
MessagePublisher::ToPointCloud2(std::vector<Vector3D> data) {
  sensor_msgs::PointCloud2 cloud;

  cloud.header.stamp = ros::Time::now();
  cloud.header.frame_id = m_frame_id;

  cloud.width = data.size();
  cloud.height = 1;

  // Convert x/y/z to fields
  cloud.fields.resize(3);
  cloud.fields[0].name = "x";
  cloud.fields[1].name = "y";
  cloud.fields[2].name = "z";

  int offset = 0;
  for (size_t d = 0; d < cloud.fields.size(); d++, offset += 4) {
    cloud.fields[d].offset = offset;
    cloud.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[d].count = 1;
  }

  cloud.point_step = offset;
  cloud.row_step = cloud.point_step * cloud.width;

  cloud.data.resize(data.size() * cloud.point_step);
  cloud.is_bigendian = false;
  cloud.is_dense = false;

  for (size_t cp = 0; cp < data.size(); cp++) {
    memcpy(&cloud.data[cp * cloud.point_step + cloud.fields[0].offset],
           &data[cp].x, sizeof(float));
    memcpy(&cloud.data[cp * cloud.point_step + cloud.fields[1].offset],
           &data[cp].y, sizeof(float));
    memcpy(&cloud.data[cp * cloud.point_step + cloud.fields[2].offset],
           &data[cp].z, sizeof(float));
  }

  return cloud;
}

sensor_msgs::NavSatFix MessagePublisher::ToNavSatFix(Location data) {
  sensor_msgs::NavSatFix nsf;

  nsf.header.stamp = ros::Time::now();
  nsf.header.frame_id = m_frame_id;

  nsf.latitude = data.latitude;
  nsf.longitude = data.longitude;
  nsf.altitude = data.altitude;

  nsf.position_covariance[0] = 0.0;
  nsf.position_covariance[4] = 0.0;
  nsf.position_covariance[8] = 0.0;

  return nsf;
}

sensor_msgs::Imu MessagePublisher::ToImu(IMU data) {
  sensor_msgs::Imu imu;

  imu.header.stamp = ros::Time::now();
  imu.header.frame_id = m_frame_id;

  imu.linear_acceleration.x = data.linear_acceleration.x;
  imu.linear_acceleration.y = data.linear_acceleration.y;
  imu.linear_acceleration.z = data.linear_acceleration.z;
  imu.linear_acceleration_covariance[0] = 2.0;
  imu.linear_acceleration_covariance[3] = 2.0;
  imu.linear_acceleration_covariance[6] = 2.0;

  imu.angular_velocity.x = data.angular_velocity.x;
  imu.angular_velocity.y = data.angular_velocity.y;
  imu.angular_velocity.z = data.angular_velocity.z;
  imu.angular_velocity_covariance[0] = 2.0;
  imu.angular_velocity_covariance[3] = 2.0;
  imu.angular_velocity_covariance[6] = 2.0;

  return imu;
}

chrono_ros_interface::Cones
MessagePublisher::ToCones(std::vector<Vector3D> blue_cones,
                          std::vector<Vector3D> yellow_cones) {
  chrono_ros_interface::Cones cones;
  int num = blue_cones.size();

  cones.header.stamp = ros::Time::now();
  cones.header.frame_id = m_frame_id;

  cones.size.data = num;

  cones.blue_cones.resize(num);
  cones.yellow_cones.resize(num);
  for (int i = 0; i < num; i++) {

    cones.blue_cones[i].pose.position.x = blue_cones[i].x;
    cones.blue_cones[i].pose.position.y = blue_cones[i].y;
    cones.blue_cones[i].pose.position.z = blue_cones[i].z;

    cones.yellow_cones[i].pose.position.x = yellow_cones[i].x;
    cones.yellow_cones[i].pose.position.y = yellow_cones[i].y;
    cones.yellow_cones[i].pose.position.z = yellow_cones[i].z;
  }

  return cones;
}
