#include "ch_ros/publishers.h"

// --------------------------------- CAMERA ----------------------------------
// //
Camera::Camera(ros::NodeHandle n, std::string node_name, int queue_size)
    : Publisher(n, node_name, 0) {
  image_transport::ImageTransport it(n);
  image_pub_ = it.advertise(node_name, queue_size);
}

void Camera::publish(const ChRosMessage::Message *message, int received) {
  // // Parse buffer
  const ChRosMessage::Camera *camera =
      static_cast<const ChRosMessage::Camera *>(message->message());

  data_.header.stamp = ros::Time::now();
  data_.header.frame_id = "base_link";

  data_.height = camera->height();
  data_.width = camera->width();

  data_.encoding = sensor_msgs::image_encodings::RGBA8;

  int num = 1; // for endianness detection
  data_.is_bigendian = !(*(char *)&num == 1);

  data_.step = camera->bytes_per_pixel() * data_.width;

  size_t size = data_.width * data_.height * camera->bytes_per_pixel();
  data_.data.resize(size);
  memcpy((uint8_t *)(&data_.data[0]), camera->points()->Data(), size);
  // rev_memcpy((uint8_t *)(&data_.data[0]), camera->points()->Data() + size,
  // size);
  // &data_.data[0] = camera->points()->Data();

  image_pub_.publish(data_);
}

// --------------------------------- LIDAR ---------------------------------- //
Lidar::Lidar(ros::NodeHandle n, std::string node_name, int queue_size)
    : Publisher(n, node_name, queue_size) {}

void Lidar::publish(const ChRosMessage::Message *message, int received) {
  // Parse buffer
  const ChRosMessage::Lidar *lidar =
      static_cast<const ChRosMessage::Lidar *>(message->message());

  data_.header.stamp = ros::Time::now();
  data_.header.frame_id = "base_link";

  data_.width = lidar->points()->Length();
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
    float x = lidar->points()->Get(cp)->x();
    float y = lidar->points()->Get(cp)->y();
    float z = lidar->points()->Get(cp)->z();

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

void IMU::publish(const ChRosMessage::Message *message, int received) {
  // Parse buffer
  const ChRosMessage::IMU *imu =
      static_cast<const ChRosMessage::IMU *>(message->message());

  // Convert to Imu
  data_.header.stamp = ros::Time::now();
  data_.header.frame_id = "base_link";

  data_.linear_acceleration.x = imu->linear_acceleration()->x();
  data_.linear_acceleration.y = imu->linear_acceleration()->y();
  data_.linear_acceleration.z = imu->linear_acceleration()->z();

  data_.angular_velocity.x = imu->angular_velocity()->x();
  data_.angular_velocity.y = imu->angular_velocity()->y();
  data_.angular_velocity.z = imu->angular_velocity()->z();

  pub_.publish(data_);
}

// --------------------------------- GPS ---------------------------------- //
GPS::GPS(ros::NodeHandle n, std::string node_name, int queue_size)
    : Publisher(n, node_name, queue_size) {}

void GPS::publish(const ChRosMessage::Message *message, int received) {
  // Parse buffer
  const ChRosMessage::GPS *gps =
      static_cast<const ChRosMessage::GPS *>(message->message());

  // Convert to NavSatFix
  data_.header.stamp = ros::Time::now();
  data_.header.frame_id = "map";

  data_.latitude = gps->latitude();
  data_.longitude = gps->longitude();
  data_.altitude = gps->altitude();

  data_.position_covariance[0] = 0.0;
  data_.position_covariance[4] = 0.0;
  data_.position_covariance[8] = 0.0;

  pub_.publish(data_);
}

// --------------------------------- TIME ---------------------------------- //
Time::Time(ros::NodeHandle n, std::string node_name, int queue_size)
    : Publisher(n, node_name, queue_size), time_(0) {}

void Time::publish(const ChRosMessage::Message *message, int received) {
  // Parse buffer
  const ChRosMessage::Time *time =
      static_cast<const ChRosMessage::Time *>(message->message());

  time_ = time->t();
  data_.clock = ros::Time(time_);

  pub_.publish(data_);
}

// --------------------------------- CONES ---------------------------------- //
Cones::Cones(ros::NodeHandle n, std::string node_name, int queue_size)
    : Publisher(n, node_name, queue_size),
      srv_(n.advertiseService("cone_map", &Cones::send_cones, this)) {}

void Cones::publish(const ChRosMessage::Message *message, int received) {
  // Parse buffer
  const ChRosMessage::Cones *cones =
      static_cast<const ChRosMessage::Cones *>(message->message());

  data_.blue_cones.resize(cones->blue_cones()->Length());
  data_.yellow_cones.resize(cones->yellow_cones()->Length());
  data_.orange_cones.resize(2);
  for (int i = 0; i < cones->blue_cones()->Length(); i++) {
    data_.blue_cones[i].position.x = cones->blue_cones()->Get(i)->x();
    data_.blue_cones[i].position.y = cones->blue_cones()->Get(i)->y();
    data_.blue_cones[i].position.z = cones->blue_cones()->Get(i)->z();
    data_.blue_cones[i].color = common_msgs::Cone::BLUE;

    data_.yellow_cones[i].position.x = cones->yellow_cones()->Get(i)->x();
    data_.yellow_cones[i].position.y = cones->yellow_cones()->Get(i)->y();
    data_.yellow_cones[i].position.z = cones->yellow_cones()->Get(i)->z();

    // if (i < 2) {
    //   data_.orange_cones[i].position.x = message.orange_cones(i).x();
    //   data_.orange_cones[i].position.y = message.orange_cones(i).y();
    //   data_.orange_cones[i].position.z = message.orange_cones(i).z();
    //   data_.orange_cones[i].color = common_msgs::Cone::ORANGE;
    // }
  }
  data_received_ = true;

  pub_.publish(data_);
}

bool Cones::send_cones(common_srvs::ConeMap::Request &req,
                       common_srvs::ConeMap::Response &res) {
  if (!data_received_)
    return false;
  res.blue_cones = data_.blue_cones;
  res.yellow_cones = data_.yellow_cones;
  res.orange_cones = data_.orange_cones;
  return true;
}

// --------------------------------- VEHICLE ----------------------------------
// //
Vehicle::Vehicle(ros::NodeHandle n, std::string node_name, int queue_size)
    : Publisher(n, node_name, queue_size) {}

void Vehicle::publish(const ChRosMessage::Message *message, int received) {
  // Parse buffer
  const ChRosMessage::Vehicle *vehicle =
      static_cast<const ChRosMessage::Vehicle *>(message->message());

  data_.header.stamp = ros::Time::now();
  data_.header.frame_id = "odom";

  data_.state.position.x = vehicle->position()->x();
  data_.state.position.y = vehicle->position()->y();
  data_.state.position.z = vehicle->position()->z();

  pub_.publish(data_);
}
