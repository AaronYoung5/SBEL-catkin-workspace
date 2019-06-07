#include "ch_ros/publishers.h"

// --------------------------------- LIDAR ---------------------------------- //
Lidar::Lidar(ros::NodeHandle n, std::string node_name, int queue_size)
    : Publisher(n, node_name, queue_size) {}

void Lidar::publish(std::vector<uint8_t> buffer, int received) {
  // Parse buffer
  ChronoMessages::lidar message;
  message.ParseFromArray(buffer.data() + 2, received - 2);

  data_.header.stamp = ros::Time::now();
  data_.header.frame_id = "lidar";

  data_.width = message.points_size();
  data_.height = 1;

  // Convert x/y/z to fields
  data_.fields.resize(3);
  data_.fields[0].name = "x";
  data_.fields[1].name = "y";
  data_.fields[2].name = "z";

  int offset = 0;
  for (size_t d = 0; d < data_.fields.size(); d++, offset += 4) {
    data_.fields[d].offset = offset;
    data_.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
    data_.fields[d].count = 1;
  }

  data_.point_step = offset;
  data_.row_step = data_.point_step * data_.width;

  data_.data.resize(data_.row_step);
  data_.is_bigendian = false;
  data_.is_dense = false;

  for (size_t cp = 0; cp < data_.width; cp++) {
    float x = message.points(cp).x();
    float y = message.points(cp).y();
    float z = message.points(cp).z();

    memcpy(&data_.data[cp * data_.point_step + data_.fields[0].offset], &x,
           sizeof(float));
    memcpy(&data_.data[cp * data_.point_step + data_.fields[1].offset], &y,
           sizeof(float));
    memcpy(&data_.data[cp * data_.point_step + data_.fields[2].offset], &z,
           sizeof(float));
  }

  pub_.publish(data_);
}

// --------------------------------- IMU ---------------------------------- //
IMU::IMU(ros::NodeHandle n, std::string node_name, int queue_size)
    : Publisher(n, node_name, queue_size) {}

void IMU::publish(std::vector<uint8_t> buffer, int received) {
  // Parse buffer
  ChronoMessages::imu message;
  message.ParseFromArray(buffer.data() + 1, received - 1);

  // Convert to Imu
  data_.header.stamp = ros::Time::now();
  data_.header.frame_id = "imu";

  data_.linear_acceleration.x = message.linear_acceleration().x();
  data_.linear_acceleration.y = message.linear_acceleration().y();
  data_.linear_acceleration.z = message.linear_acceleration().z();

  data_.angular_velocity.x = message.angular_velocity().x();
  data_.angular_velocity.y = message.angular_velocity().y();
  data_.angular_velocity.z = message.angular_velocity().z();

  pub_.publish(data_);
}

// --------------------------------- GPS ---------------------------------- //
GPS::GPS(ros::NodeHandle n, std::string node_name, int queue_size)
    : Publisher(n, node_name, queue_size) {}

void GPS::publish(std::vector<uint8_t> buffer, int received) {
  // Parse buffer
  ChronoMessages::gps message;
  message.ParseFromArray(buffer.data() + 1, received - 1);

  // Convert to NavSatFix
  data_.header.stamp = ros::Time::now();
  data_.header.frame_id = "gps";

  data_.latitude = message.latitude();
  data_.longitude = message.longitude();
  data_.altitude = message.altitude();

  data_.position_covariance[0] = 0.0;
  data_.position_covariance[4] = 0.0;
  data_.position_covariance[8] = 0.0;

  pub_.publish(data_);
}

// --------------------------------- TIME ---------------------------------- //
Time::Time(ros::NodeHandle n, std::string node_name, int queue_size)
    : Publisher(n, node_name, queue_size) {}

void Time::publish(std::vector<uint8_t> buffer, int received) {
  // Parse buffer
  ChronoMessages::time message;
  message.ParseFromArray(buffer.data() + 1, received - 1);
  data_.clock = ros::Time(message.t());

  pub_.publish(data_);
}

// --------------------------------- CONES ---------------------------------- //
Cones::Cones(ros::NodeHandle n, std::string node_name, int queue_size)
    : Publisher(n, node_name, queue_size) {}

void Cones::publish(std::vector<uint8_t> buffer, int received) {
  // Parse buffer
  ChronoMessages::cones message;
  message.ParseFromArray(buffer.data() + 1, received - 1);

  data_.blue_cones.resize(message.blue_cones_size());
  data_.yellow_cones.resize(message.yellow_cones_size());
  data_.orange_cones.resize(2);
  for (int i = 0; i < message.blue_cones_size(); i++) {
    data_.blue_cones[i].position.x = message.blue_cones(i).x();
    data_.blue_cones[i].position.y = message.blue_cones(i).y();
    data_.blue_cones[i].position.z = message.blue_cones(i).z();
    data_.blue_cones[i].color = common_msgs::Cone::BLUE;

    data_.yellow_cones[i].position.x = message.yellow_cones(i).x();
    data_.yellow_cones[i].position.y = message.yellow_cones(i).y();
    data_.yellow_cones[i].position.z = message.yellow_cones(i).z();
    data_.yellow_cones[i].color = common_msgs::Cone::YELLOW;

    if (i < 2) {
      data_.orange_cones[i].position.x = message.orange_cones(i).x();
      data_.orange_cones[i].position.y = message.orange_cones(i).y();
      data_.orange_cones[i].position.z = message.orange_cones(i).z();
      data_.orange_cones[i].color = common_msgs::Cone::ORANGE;
    }
  }
}
