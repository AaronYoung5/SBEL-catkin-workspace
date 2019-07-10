// Header file include
#include "pid_control/PID.h"

PID::PID(ros::NodeHandle n, float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd), p_error_(0), d_error_(0), i_error_(0),
      prev_error_(0), throttle_(0), braking_(0), steering_(0),
      controls_pub_(n.advertise<common_msgs::Control>("control", 10)),
      point_following_pub_(
          n.advertise<geometry_msgs::Point>("point_following", 10)),
      // cone_sub_(n.subscribe("cones", 10, &PID::coneCallback, this)),
      state_sub_(n.subscribe("vehicle", 10, &PID::stateCallback, this)),
      client_(n.serviceClient<common_srvs::ConeMap>("cone_map", true)),
      srv_(n.advertiseService("path", &PID::pathCallback, this)) {
  
}

void PID::coneCallback(const common_msgs::ConeMap::ConstPtr &msg) {
  // cone_map_ = *msg;
  // cone_map_received_ = 1;
}

void PID::stateCallback(const common_msgs::VehState::ConstPtr &msg) {
  // float x = msg->state.position.x;
  // float y = msg->state.position.y;
  // float z = msg->state.position.z;

  Vector3D<> pos(msg->state.position.x, msg->state.position.y,
                 msg->state.position.z);

  float min_dist = 1000;
  float min_index = 0;
  for (int i = 0; i < path_.size() - 1; i++) {
    float temp_dist = minimumDistance(pos, path_[i], path_[i + 1], false);
    if (abs(temp_dist) < abs(min_dist)) {
      // std::cout << "Temp Distance :: " << temp_dist << std::endl;
      // std::cout << "Min Distance :: " << min_dist << std::endl;
      // std::cout << "Current Position :: "
      // << "(" << pos.x() << ", " << pos.y() << ")" << std::endl;
      // std::cout << "Path Position :: "
      // << "(" << path_[i].x() << ", " << path_[i].y() << ")" << std::endl;
      // std::cout << "Index :: " << i << std::endl;
      min_dist = temp_dist;
      min_index = i;
    }
  }
  minimumDistance(pos, path_[min_index], path_[min_index + 1], true);
  // std::cout << "Min Distance :: " << min_dist << std::endl;

  // min_dist =
  //     distance(path_[min_index + 1].x, path_[min_index + 1].y, 0, x, y, 0);

  // std::cout << "Min Distance :: " << min_dist << std::endl;
  // std::cout << "Index :: " << min_index << std::endl;

  updateError(min_dist);
  float total_error = totalError();

  //Release the gas pedal if the deviation from the lane's center is high
  throttle_value = speed_pid.TotalError() - 1.25*cte*cte;
  if(throttle_value<-0.01){
    throttle_value=-0.01;
  }
  throttle_ = .25;
  braking_ = 0;
  steering_ = total_error;

  publish();
}

void PID::publish() {
  clampControls();

  common_msgs::Control control;
  control.throttle.data = throttle_;
  control.braking.data = braking_;
  control.steering.data = steering_;

  // std::cout << "Steering :: " << steering_ << std::endl;

  controls_pub_.publish(control);
}

float PID::minimumDistance(Vector3D<> pos, Vector3D<> begin, Vector3D<> end,
                           bool publish) {
  const float len = (begin - end).length();
  if (len == 0.0)
    return (pos - begin).length(); // Case where begin == end
  const float t = std::max(0.0f, std::min(1.0f, (pos - begin).dot(end - begin) /
                                                    (float)pow(len, 2)));
  Vector3D<> projection = begin + (end - begin) * t;
  float dist = (pos - projection).length();
  int sign = std::signbit((projection.x() - begin.x()) * (pos.y() - begin.y()) -
                          (projection.y() - begin.y()) * (pos.x() - begin.x()));
  if (publish)
    pointFollowingPublish(projection);
  // std::cout << "SIGN :: " << sign << std::endl;
  return sign ? dist : -dist;
}

// float PID::minimumDistance(Vector3D<> pos, Vector3D<> begin, Vector3D<> end,
//                            bool publish) {
//   Vector3D<> segment = end - begin;
//   Vector3D<> projection =
//       segment.getUnit() * (pos - begin).dot(segment.getUnit());
//   if (publish) {
//     pointFollowingPublish(projection + begin);
//     return (pos - begin).cross(projection + begin).z() > 0 ? -(pos -
//     projection).length()
//                                          : (pos - projection).length();
//   }
//   return (pos - projection).length();
// }

void PID::pointFollowingPublish(Vector3D<> pos) {
  geometry_msgs::Point point;
  point.x = pos.x();
  point.y = pos.y();
  point.z = pos.z();

  point_following_pub_.publish(point);
}

void PID::clampControls() {
  throttle_ = throttle_ > 1 ? 1 : throttle_ < 0 ? 0 : throttle_;
  steering_ = steering_ > 1 ? 1 : steering_ < -1 ? -1 : steering_;
  braking_ = braking_ > 1 ? 1 : braking_ < 0 ? 0 : braking_;
}

void PID::updateError(float cte) {
  // std::cout << "Cross Track Error :: " << cte << std::endl;
  d_error_ = cte - p_error_;
  p_error_ = cte;
  i_error_ += cte;
}

float PID::totalError() {
  // std::cout << "-" << kp_ << " * " << p_error_ << " - " << kd_ << " * "
  // << d_error_ << " - " << ki_ << " * " << i_error_ <<
  // "\tTotalError: "
  // << -kp_ * p_error_ - kd_ * d_error_ - ki_ * i_error_ <<
  // std::endl;
  // std::cout << "Total Error :: "
  // << -kp_ * p_error_ - kd_ * d_error_ - ki_ * i_error_ << std::endl;

  return -kp_ * p_error_ - kd_ * d_error_ - ki_ * i_error_;
}

// void PID::printVec(std::string title, Vector3D<> &vec) {
//   std::cout << title << " :: (" << vec.x() << ", " << vec.y() << ", " <<
//   vec.z()
//             << ")" << std::endl;
// }

bool PID::pathCallback(common_srvs::Path::Request &req,
                       common_srvs::Path::Response &res) {
  for (Vector3D<> vec : path_) {
    geometry_msgs::Point p;
    p.x = vec.x();
    p.y = vec.y();
    p.z = vec.z();
    res.points.push_back(p);
  }
  return true;
}
