#include "ch_ros/publishers.h"

// --------------------------------- LIDAR ---------------------------------- //
Lidar::Lidar(ros::NodeHandle n, std::string node_name, int queue_size)
    : Publisher(n, node_name, queue_size) {}

void Lidar::publish(std::vector<uint8_t> buffer, int received) {
  // Parse buffer
  ChronoMessages::lidar message;
  message.ParseFromArray(buffer.data() + 1, received - 1);

  int id = message.id();
  // std::cout << id << std::endl;
  if (id != (last_id_ + 1)) {
    std::cout << "Packet Lost :: "
              << "id(" << id << "), last_id(" << last_id_ << ")" << std::endl;
  }
  last_id_ = id;

  if (message.expected()) {
    expected_ = message.expected();

    data_.width = 0;
    data_.data.clear();
    data_.row_step = 0;

    data_.header.stamp = ros::Time::now();
    data_.header.frame_id = "base_link";

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

    data_.is_bigendian = false;
    data_.is_dense = false;
  }

  int old_size = data_.width;

  data_.width = data_.width + message.points_size();
  data_.row_step = data_.point_step * message.points_size();

  data_.data.resize(data_.data.size() + data_.row_step);

  for (size_t cp = old_size; cp < data_.width; cp++) {
    float x = message.points(cp - old_size).x();
    float y = message.points(cp - old_size).y();
    float z = message.points(cp - old_size).z();

    memcpy(&data_.data[cp * data_.point_step + data_.fields[0].offset], &x,
           sizeof(float));
    memcpy(&data_.data[cp * data_.point_step + data_.fields[1].offset], &y,
           sizeof(float));
    memcpy(&data_.data[cp * data_.point_step + data_.fields[2].offset], &z,
           sizeof(float));
  }

  // std::cout << expected_ << std::endl;
  if (expected_ == message.num()) {
    expected_ = 0;
    pub_.publish(data_);
  }
  // cur_id_ = message.id();
}

void Lidar::tcppublish(std::vector<uint8_t> buffer, int received) {
  if (use_protobuf) {
    // Parse buffer
    ChronoMessages::lidar message;
    message.ParseFromArray(buffer.data() + 1, received - 1);

    data_.header.stamp = ros::Time::now();
    data_.header.frame_id = "base_link";

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
  } else {
    // Parse buffer
    const RosMessage::lidar *message =
        flatbuffers::GetRoot<RosMessage::lidar>(buffer.data());
    flatbuffers::Verifier verifier(
        reinterpret_cast<unsigned char *>(buffer.data()), received);
    std::cout << message->Verify(verifier) << std::endl;

    data_.header.stamp = ros::Time::now();
    data_.header.frame_id = "base_link";

    data_.width = message->points()->Length();
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
      float x = message->points()->Get(cp)->x();
      float y = message->points()->Get(cp)->y();
      float z = message->points()->Get(cp)->z();

      memcpy(&data_.data[cp * data_.point_step + data_.fields[0].offset], &x,
             sizeof(float));
      memcpy(&data_.data[cp * data_.point_step + data_.fields[1].offset], &y,
             sizeof(float));
      memcpy(&data_.data[cp * data_.point_step + data_.fields[2].offset], &z,
             sizeof(float));
    }

    pub_.publish(data_);
  }
}

// --------------------------------- IMU ---------------------------------- //
IMU::IMU(ros::NodeHandle n, std::string node_name, int queue_size)
    : Publisher(n, node_name, queue_size) {}

void IMU::publish(std::vector<uint8_t> buffer, int received) {
  if (use_protobuf) {
    // Parse buffer
    ChronoMessages::imu message;
    message.ParseFromArray(buffer.data() + 1, received - 1);

    // Convert to Imu
    data_.header.stamp = ros::Time::now();
    data_.header.frame_id = "base_link";

    data_.linear_acceleration.x = message.linear_acceleration().x();
    data_.linear_acceleration.y = message.linear_acceleration().y();
    data_.linear_acceleration.z = message.linear_acceleration().z();

    data_.angular_velocity.x = message.angular_velocity().x();
    data_.angular_velocity.y = message.angular_velocity().y();
    data_.angular_velocity.z = message.angular_velocity().z();

    pub_.publish(data_);
  } else {
    // Parse buffer
    const RosMessage::imu *message =
        flatbuffers::GetRoot<RosMessage::imu>(buffer.data());

    // Convert to Imu
    data_.header.stamp = ros::Time::now();
    data_.header.frame_id = "base_link";

    data_.linear_acceleration.x = message->linear_acceleration()->x();
    data_.linear_acceleration.y = message->linear_acceleration()->y();
    data_.linear_acceleration.z = message->linear_acceleration()->z();

    data_.angular_velocity.x = message->angular_velocity()->x();
    data_.angular_velocity.y = message->angular_velocity()->y();
    data_.angular_velocity.z = message->angular_velocity()->z();
  }
}

// --------------------------------- GPS ---------------------------------- //
GPS::GPS(ros::NodeHandle n, std::string node_name, int queue_size)
    : Publisher(n, node_name, queue_size) {}

void GPS::publish(std::vector<uint8_t> buffer, int received) {
  if (use_protobuf) {
    // Parse buffer
    ChronoMessages::gps message;
    message.ParseFromArray(buffer.data() + 1, received - 1);

    // Convert to NavSatFix
    data_.header.stamp = ros::Time::now();
    data_.header.frame_id = "map";

    data_.latitude = message.latitude();
    data_.longitude = message.longitude();
    data_.altitude = message.altitude();

    data_.position_covariance[0] = 0.0;
    data_.position_covariance[4] = 0.0;
    data_.position_covariance[8] = 0.0;

    pub_.publish(data_);
  } else {
    // Parse buffer
    std::cout << "Test 1" << std::endl;
    const RosMessage::gps *message =
        flatbuffers::GetRoot<RosMessage::gps>(buffer.data());
    std::cout << "Test 2" << std::endl;

    // Convert to NavSatFix
    data_.header.stamp = ros::Time::now();
    data_.header.frame_id = "map";

    data_.latitude = message->latitude();
    std::cout << "Test 3" << std::endl;
    data_.longitude = message->longitude();
    data_.altitude = message->altitude();

    data_.position_covariance[0] = 0.0;
    data_.position_covariance[4] = 0.0;
    data_.position_covariance[8] = 0.0;

    pub_.publish(data_);
  }
}

// --------------------------------- TIME ---------------------------------- //
Time::Time(ros::NodeHandle n, std::string node_name, int queue_size)
    : Publisher(n, node_name, queue_size) {}

void Time::publish(std::vector<uint8_t> buffer, int received) {
  if (use_protobuf) {
    // Parse buffer
    ChronoMessages::time message;
    message.ParseFromArray(buffer.data() + 1, received - 1);
    data_.clock = ros::Time(message.t());

    pub_.publish(data_);
  } else {
    // Parse buffer
    const RosMessage::time *message =
        flatbuffers::GetRoot<RosMessage::time>(buffer.data());

    data_.clock = ros::Time(message->t());

    pub_.publish(data_);
  }
}

// --------------------------------- CONES ---------------------------------- //
Cones::Cones(ros::NodeHandle n, std::string node_name, int queue_size)
    : Publisher(n, node_name, queue_size) {}

void Cones::publish(std::vector<uint8_t> buffer, int received) {
  if (use_protobuf) {
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

      // if (i < 2) {
      //   data_.orange_cones[i].position.x = message.orange_cones(i).x();
      //   data_.orange_cones[i].position.y = message.orange_cones(i).y();
      //   data_.orange_cones[i].position.z = message.orange_cones(i).z();
      //   data_.orange_cones[i].color = common_msgs::Cone::ORANGE;
      // }
    }
  } else {
    const RosMessage::cones *message =
        flatbuffers::GetRoot<RosMessage::cones>(buffer.data());

    data_.blue_cones.resize(message->blue_cones()->Length());
    data_.yellow_cones.resize(message->yellow_cones()->Length());
    data_.orange_cones.resize(2);
    for (int i = 0; i < message->blue_cones()->Length(); i++) {
      data_.blue_cones[i].position.x = message->blue_cones()->Get(i)->x();
      data_.blue_cones[i].position.y = message->blue_cones()->Get(i)->y();
      data_.blue_cones[i].position.z = message->blue_cones()->Get(i)->z();
      data_.blue_cones[i].color = common_msgs::Cone::BLUE;

      data_.yellow_cones[i].position.x = message->yellow_cones()->Get(i)->x();
      data_.yellow_cones[i].position.y = message->yellow_cones()->Get(i)->y();
      data_.yellow_cones[i].position.z = message->yellow_cones()->Get(i)->z();
      data_.yellow_cones[i].color = common_msgs::Cone::YELLOW;

      // if (i < 2) {
      //   data_.orange_cones[i].position.x = message.orange_cones(i).x();
      //   data_.orange_cones[i].position.y = message.orange_cones(i).y();
      //   data_.orange_cones[i].position.z = message.orange_cones(i).z();
      //   data_.orange_cones[i].color = common_msgs::Cone::ORANGE;
      // }
    }
  }
}
